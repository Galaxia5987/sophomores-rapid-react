package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.hopper
import frc.robot.lib.onTrue
import frc.robot.roller
import frc.robot.subsystems.leds.StateColor
import frc.robot.subsystems.leds.applyPattern
import org.littletonrobotics.junction.Logger

private val ballsEmpty = roller.hasBall.negate().and(hopper.hasBall.negate())
private val isShooting = Trigger { state == RobotState.SHOOTING }
private val isInDeadZone =
    Trigger {
        robotDistanceFromBasket in
                MIN_DISTANCE_FROM_BASKET..MAX_DISTANCE_FROM_BASKET
    }
        .negate()

private val isIntaking = Trigger { state == RobotState.INTAKING }
private val hasFrontBall = roller.hasBall
private val hasBackBall = hopper.hasBall

val RobotCommandsLogger
    get() = {
        Logger.recordOutput("$COMMAND_NAME_PREFIX/RobotState", state)
        Logger.recordOutput(
            "$COMMAND_NAME_PREFIX/RobotDistanceFromHub",
            robotDistanceFromBasket
        )
        Logger.recordOutput("$COMMAND_NAME_PREFIX/is in dead zone", isInDeadZone)
        Logger.recordOutput("$COMMAND_NAME_PREFIX/fly wheel target velocity", flywheelTargetVelocity)
        Logger.recordOutput("$COMMAND_NAME_PREFIX/hoodRotation", hoodAngle)
    }

fun bindRobotCommands() {
    isShooting.apply {
        onTrue(StateColor.SHOOTING.applyPattern())
        and(ballsEmpty).onTrue(setIntakeing(), stopShooting())
        and(isInDeadZone.negate()).onTrue(shooting())
        and(isInDeadZone).onTrue(driveToShootingPoint())
    }
    isIntaking.apply {
        onTrue(StateColor.INTAKING.applyPattern())
        and(ballsEmpty).onTrue(intaking())
        and(hasFrontBall)
            .and(hasBackBall)
            .onTrue(roller.stop(), hopper.stop(), setShooting())
        and(hasBackBall).and(hasFrontBall.negate()).onTrue(hopper.stop())
    }
}

private fun setRobotState(newStates: RobotState) =
    Commands.runOnce({ state = newStates })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntakeing() = setRobotState(RobotState.INTAKING)
