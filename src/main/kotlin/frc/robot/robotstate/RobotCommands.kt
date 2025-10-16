package frc.robot.robotstate

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import frc.robot.*
import frc.robot.lib.extensions.*
import frc.robot.lib.getPose2d
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.lib.named
import frc.robot.lib.shooting.ShotData
import frc.robot.lib.shooting.calculateShot
import frc.robot.robotRelativeBallPoses
import frc.robot.subsystems.drive.align
import frc.robot.subsystems.drive.profiledAlign
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.shooter.flywheel.*
import frc.robot.subsystems.shooter.hood.HOOD_ANGLE_BY_DISTANCE
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.hopper.Hopper
import frc.robot.subsystems.shooter.turret.MAX_ANGLE
import frc.robot.subsystems.shooter.turret.MIN_ANGLE
import kotlin.collections.map
import org.team5987.annotation.LoggedOutput

var hoodAngle = InterpolatingDouble(robotDistanceFromHub[m])
var forceShoot = false
var overrideDrive = false

val compensatedShot: ShotData
    get() {
        val robotSpeeds =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                drive.chassisSpeeds,
                drive.rotation
            )
        val shooterExitVelocity =
            Flywheel.velocity.toLinear(FLYWHEEL_DIAMETER, 1.0)
        val shot = calculateShot(drive.pose, robotSpeeds, shooterExitVelocity)

        mapOf(
                "compensatedShot/compensatedTarget" to
                    Pose2d(shot.compensatedTarget, Rotation2d()),
                "regularShot/target" to Pose2d(HUB_LOCATION, Rotation2d()),
                "compensatedShot/compensatedDistance" to
                    shot.compensatedDistance,
                "regularShot/distance" to robotDistanceFromHub,
                "compensatedShot/turretAngle" to shot.turretAngle.measure,
                "regularShot/turretAngle" to angleFromRobotHub,
                "shooterExitVelocity" to shooterExitVelocity
            )
            .log("onMoveShoot")

        return shot
    }

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val robotDistanceFromHub
    get() = drive.pose.distanceFromPoint(HUB_LOCATION)

val angleFromRobotHub
    get() =
        (drive.pose.translation.rotationToPoint(HUB_LOCATION) -
                drive.pose.rotation)
            .measure

val turretAngleToHub: Angle
    get() = compensatedShot.turretAngle.measure.coerceIn(MIN_ANGLE, MAX_ANGLE)

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val swerveCompensationAngle
    get() = drive.rotation + Rotation2d(angleFromRobotHub - turretAngleToHub)

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val globalBallPoses
    get() =
        robotRelativeBallPoses
            .map { it + Pose3d(drive.pose).toTransform() }
            .toTypedArray()

fun setForceShot() = Commands.runOnce({ forceShoot = true })

fun stopForceShot() = Commands.runOnce({ forceShoot = false })

fun setOverrideDrive() = Commands.runOnce({ overrideDrive = true })

fun stopOverrideDrive() = Commands.runOnce({ overrideDrive = false })

fun driveToShootingPoint(): Command {
    val robotTranslation = drive.pose.translation
    val setpoint =
        if (
            INNER_SHOOTING_AREA.getDistance(robotTranslation) <
                OUTER_SHOOTING_AREA.getDistance(robotTranslation)
        )
            INNER_SHOOTING_AREA.nearest(robotTranslation)
        else OUTER_SHOOTING_AREA.nearest(robotTranslation)
    return align(getPose2d(setpoint, swerveCompensationAngle)).named("Drive")
}

fun startShooting() =
    sequence(
            drive.lock(),
            Flywheel.setVelocity {
                FLYWHEEL_VELOCITY_KEY.value = robotDistanceFromHub[m]
                SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(
                        FLYWHEEL_VELOCITY_KEY
                    )
                    .value
                    .rps
            },
            waitUntil(Flywheel.isAtSetVelocity),
            parallel(Hopper.start(), Roller.intake())
        )
        .named(COMMAND_NAME_PREFIX)

fun stopShooting() =
    parallel(Flywheel.setVelocity(SLOW_ROTATION), Hopper.stop(), Roller.stop())
        .named(COMMAND_NAME_PREFIX)

fun stopIntaking() =
    parallel(Roller.stop(), Hopper.stop()).named(COMMAND_NAME_PREFIX)

fun alignToBall(toRun: Boolean = true): Command {
    return if (toRun)
        profiledAlign(globalBallPoses.firstOrNull()?.toPose2d() ?: drive.pose)
            .named(COMMAND_NAME_PREFIX)
    else Commands.none()
}

fun hoodDefaultCommand() =
    Hood.setAngle {
        hoodAngle.value = compensatedShot.compensatedDistance[m]
        HOOD_ANGLE_BY_DISTANCE.getInterpolated(hoodAngle).value.deg
    }

fun enableDefaultCommands(): Command {
    return Commands.runOnce({
        turret.defaultCommand = turret.setAngle { turretAngleToHub }
        Hood.defaultCommand = hoodDefaultCommand()
    })
}
