package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.applyLeds
import frc.robot.drive
import frc.robot.hopper
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.onTrue
import frc.robot.roller
import org.littletonrobotics.junction.Logger

private val ballsEmpty = roller.hasBall.negate().and(hopper.hasBall.negate())
val isShooting = Trigger { state == RobotState.SHOOTING }
val isInDeadZone = Trigger {
    val driveTranslation = drive.pose.translation
    !OUTER_SHOOTING_AREA.contains(driveTranslation) ||
        INNER_SHOOTING_AREA.contains(driveTranslation)
}
val atShootingRotation = Trigger {
    drive.pose.rotation.measure.isNear(swerveAngle, ROTATION_TOLERANCE)
}

val isIntaking = Trigger { state == RobotState.INTAKING }
private val hasFrontBall = roller.hasBall
val hasBackBall = hopper.hasBall

val RobotCommandsLogger
    get() = {
        Logger.recordOutput("$COMMAND_NAME_PREFIX/RobotState", state)
        Logger.recordOutput(
            "$COMMAND_NAME_PREFIX/RobotDistanceFromHub",
            robotDistanceFromBasket
        )
        Logger.recordOutput(
            "$COMMAND_NAME_PREFIX/is in dead zone",
            isInDeadZone
        )
        Logger.recordOutput(
            "$COMMAND_NAME_PREFIX/fly wheel target velocity",
            flywheelTargetVelocity
        )
        Logger.recordOutput(
            "$COMMAND_NAME_PREFIX/turret rotation",
            turretRotationToBasket[deg]
        )
        Logger.recordOutput("$COMMAND_NAME_PREFIX/hoodRotation", hoodAngle)
    }

fun bindRobotCommands() {
    isShooting.apply {
        and(ballsEmpty).onTrue(setIntakeing(), stopShooting())
        and(isInDeadZone.negate()).and(atShootingRotation).onTrue(shooting())
        and(isInDeadZone).onTrue(driveToShootingPoint())
    }
    isIntaking.apply {
        and(ballsEmpty).onTrue(intaking())
        and(hasFrontBall)
            .and(hasBackBall)
            .onTrue(roller.stop(), hopper.stop(), setShooting())
        and(hasBackBall).and(hasFrontBall.negate()).onTrue(hopper.stop())
    }
    applyLeds()
}

private fun setRobotState(newStates: RobotState) =
    Commands.runOnce({ state = newStates })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntakeing() = setRobotState(RobotState.INTAKING)
