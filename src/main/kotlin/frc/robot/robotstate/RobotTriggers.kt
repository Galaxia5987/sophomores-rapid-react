package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.applyLeds
import frc.robot.drive
import frc.robot.hopper
import frc.robot.lib.extensions.onTrue
import frc.robot.roller
import frc.robot.turret
import org.littletonrobotics.junction.Logger.recordOutput

val isShooting = Trigger { state == RobotState.SHOOTING }
val isInDeadZone = Trigger {
    val driveTranslation = drive.pose.translation
    !OUTER_SHOOTING_AREA.contains(driveTranslation) ||
        INNER_SHOOTING_AREA.contains(driveTranslation)
}
val atShootingRotation =
    turret.isAtSetpoint.and {
        drive.pose.rotation.measure.isNear(
            swerveCompensationAngle.measure,
            ROTATION_TOLERANCE
        )
    }

val isIntaking = Trigger { state == RobotState.INTAKING }
private val hasFrontBall = roller.hasBall
val hasBackBall = hopper.hasBall
private val ballsEmpty = hasFrontBall.or(hasBackBall).negate()

fun robotCommandsLogger() {
    recordOutput("$COMMAND_NAME_PREFIX/RobotState", state)
    recordOutput(
        "$COMMAND_NAME_PREFIX/RobotDistanceFromHub",
        robotDistanceFromHub
    )
    recordOutput("$COMMAND_NAME_PREFIX/is in dead zone", isInDeadZone)
    recordOutput(
        "$COMMAND_NAME_PREFIX/Turret rotation to Hub",
        turretAngleToHub
    )
    recordOutput("$COMMAND_NAME_PREFIX/atShootingRotation", atShootingRotation)
}

fun bindRobotCommands() {
    isShooting.apply {
        and(ballsEmpty).onTrue(setIntakeing(), stopShooting())
        and(isInDeadZone.negate())
            .and(atShootingRotation)
            .onTrue(startShooting())
        and((isInDeadZone).or(atShootingRotation.negate()))
            .onTrue(driveToShootingPoint())
    }
    isIntaking.apply {
        and(ballsEmpty).onTrue(startIntaking())
        and(hasFrontBall)
            .and(hasBackBall)
            .onTrue(roller.stop(), hopper.stop(), setShooting())
        and(hasBackBall).and(hasFrontBall.negate()).onTrue(hopper.stop())
    }
    applyLeds()
}

private fun setRobotState(newState: RobotState) =
    Commands.runOnce({ state = newState })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntakeing() = setRobotState(RobotState.INTAKING)
