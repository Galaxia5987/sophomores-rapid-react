package frc.robot.robotstate

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import frc.robot.drive
import frc.robot.flywheel
import frc.robot.hopper
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.toTransform
import frc.robot.lib.getPose2d
import frc.robot.lib.named
import frc.robot.robotRelativeBallPoses
import frc.robot.roller
import frc.robot.subsystems.drive.alignToPose
import frc.robot.subsystems.shooter.flywheel.SHOOTER_VELOCITY_BY_DISTANCE
import frc.robot.subsystems.shooter.flywheel.SLOW_ROTATION
import frc.robot.subsystems.shooter.turret.MAX_ANGLE
import frc.robot.subsystems.shooter.turret.MIN_ANGLE

val robotDistanceFromHub
    get() = drive.pose.distanceFromPoint(HUB_LOCATION)

val angleFromRobotHub
    get() =
        (drive.pose.translation.rotationToPoint(HUB_LOCATION) -
                drive.pose.rotation)
            .measure

val turretAngleToHub: Angle
    get() = angleFromRobotHub.coerceIn(MIN_ANGLE, MAX_ANGLE)

val swerveCompensationAngle
    get() = drive.rotation + Rotation2d(angleFromRobotHub - turretAngleToHub)

val globalBallPoses
    get() =
        robotRelativeBallPoses.map { drive.pose + it.toPose2d().toTransform() }

fun driveToShootingPoint() =
    drive
        .defer {
            val robotTranslation = drive.pose.translation
            val setpoint =
                if (
                    INNER_SHOOTING_AREA.getDistance(robotTranslation) <
                        OUTER_SHOOTING_AREA.getDistance(robotTranslation)
                ) {
                    INNER_SHOOTING_AREA.nearest(robotTranslation)
                } else {
                    OUTER_SHOOTING_AREA.nearest(robotTranslation)
                }
            alignToPose((getPose2d(setpoint, swerveCompensationAngle)))
        }
        .named("Drive")

fun startShooting() =
    sequence(
            drive.lock(),
            flywheel.setVelocity {
                FLYWHEEL_VELOCITY_KEY.value = robotDistanceFromHub[m]
                SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(
                        FLYWHEEL_VELOCITY_KEY
                    )
                    .value
                    .rps
            },
            waitUntil(flywheel.isAtSetVelocity),
            parallel(hopper.start(), roller.intake())
        )
        .named(COMMAND_NAME_PREFIX)

fun stopShooting() =
    parallel(flywheel.setVelocity(SLOW_ROTATION), hopper.stop(), roller.stop())
        .named(COMMAND_NAME_PREFIX)

fun stopIntaking() =
    parallel(roller.stop(), hopper.stop()).named(COMMAND_NAME_PREFIX)

fun alignToBall() =
    drive
        .defer { alignToPose(globalBallPoses.first()) }
        .named(COMMAND_NAME_PREFIX)
