package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.hopper
import frc.robot.lib.onTrue
import frc.robot.roller
import frc.robot.subsystems.leds.StateColor
import frc.robot.subsystems.leds.applyPattern
import kotlin.and

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

fun bindRobotStateTriggers() {
    IsShooting.apply {
        and(ballsEmpty).onTrue(setIntakeing(), stopShooting())
        and(isInDeadZone.negate()).onTrue(shooting())
        and(isInDeadZone).onTrue(driveToShootingPoint())
        onTrue(StateColor.SHOOTING.applyPattern())
    }

    IsIntaking.apply {
        onTrue(StateColor.INTAKING.applyPattern())
        and(hasFrontBall).and(hasBackBall).onTrue(roller.stop(), setShooting())
        and(hasBackBall).and(hasFrontBall.negate()).onTrue(hopper.start())
        and(hasFrontBall).onTrue(roller.stop())
        and(hasBackBall.negate()).onTrue(hopper.start())
    }
}

private fun setRobotState(newStates: RobotState) =
    Commands.runOnce({ state = newStates })

fun setShooting() = setRobotState(RobotState.SHOOTING)

fun setIntakeing() = setRobotState(RobotState.INTAKING)
