package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.applyLeds
import frc.robot.drive
import frc.robot.lib.extensions.*
import frc.robot.lib.onTrue
import frc.robot.robotRelativeBallPoses
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.flywheel.STATIC_SHOOT_VELOCITY
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.hood.STATIC_SHOOT_SETPOINT
import frc.robot.subsystems.shooter.hopper.Hopper
import frc.robot.subsystems.shooter.turret.Turret
import org.team5987.annotation.LoggedOutput

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val isInDeadZone = Trigger {
    val driveTranslation = drive.pose.translation
    !OUTER_SHOOTING_AREA.contains(driveTranslation) ||
        INNER_SHOOTING_AREA.contains(driveTranslation)
}

@LoggedOutput(path = COMMAND_NAME_PREFIX)
val atShootingRotation =
    Turret.isAtSetpoint.and {
        drive.pose.rotation.measure.isNear(
            swerveCompensationAngle.measure,
            ROTATION_TOLERANCE
        )
    }

val isShooting = Trigger { state == RobotState.SHOOTING }
val isStaticShooting = Trigger { state == RobotState.STATIC_SHOOTING }
val isIntaking = Trigger { state == RobotState.INTAKING }
private val hasFrontBall = Roller.hasBall
val hasBackBall = Hopper.hasBall
private val ballsEmpty = hasFrontBall.or(hasBackBall).negate()

fun bindRobotCommands() {
    isShooting.apply {
        and(ballsEmpty.and { !forceShoot })
            .onTrue(setIntaking(), stopShooting())
        and(!isInDeadZone, atShootingRotation, { !ShootOnMove.get() })
            .onTrue(startShooting())
        and(!isInDeadZone, { ShootOnMove.get() })
            .onTrue(startShooting())
        and((isInDeadZone).or(!atShootingRotation))
            .onTrue(driveToShootingPoint())
    }
    isIntaking.apply {
        and(hasFrontBall, hasBackBall)
            .onTrue(Roller.stop(), Hopper.stop(), setShooting())
        and(hasBackBall, !hasFrontBall).apply {
            onTrue(stopIntaking())
            and(robotRelativeBallPoses::isNotEmpty, { intakeByVision }).apply {
                onTrue(Roller.intake())
                and { !forceShoot }.onTrue(alignToBall(overrideDrive))
            }
            and { !intakeByVision }.onTrue(Roller.intake())
        }
        and(ballsEmpty).apply {
            and(robotRelativeBallPoses::isNotEmpty, { intakeByVision }).apply {
                onTrue(
                    Roller.intake(),
                    Hopper.start(),
                    alignToBall(overrideDrive)
                )
            }
            onTrue(stopIntaking())
            and { !intakeByVision }.onTrue(Roller.intake(), Hopper.start())
        }
    }
    isStaticShooting.apply {
        onTrueUntil(
            this,
            Roller.intake(),
            Hopper.start(),
            Hood.setAngle { STATIC_SHOOT_SETPOINT },
            Flywheel.setVelocity { STATIC_SHOOT_VELOCITY }
        )
    }
    applyLeds()
}

private fun setRobotState(newState: RobotState) =
    Commands.runOnce({ state = newState })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntaking() = setRobotState(RobotState.INTAKING)

fun setIdling() = setRobotState(RobotState.IDLING)

fun setStaticShooting() = setRobotState(RobotState.STATIC_SHOOTING)
