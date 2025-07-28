package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.hopper
import frc.robot.lib.onTrue
import frc.robot.roller
import frc.robot.subsystems.leds.applyPattern
import frc.robot.subsystems.leds.stateColor

private val ballsEmpty = roller.HasBall.negate().and(hopper.hasBall.negate())
private val IsShooting = Trigger { state == robotState.SHOOTING }
private val isInDeadZone = Trigger {
    robotDistanceFromBasket in
        MIN_DISTANCE_FROM_BASKET..MAX_DISTANCE_FROM_BASKET
}

fun bindRobotStateTriggers() {
    IsShooting.apply {
        and(ballsEmpty).apply { onTrue(setIntakeing(), stopShooting()) }
        and(isInDeadZone.negate()).onTrue(shooting())
        and(isInDeadZone).onTrue(driveToShootingPoint())
        onTrue(stateColor.SHOOTING.applyPattern())
    }
}

private fun setRobotState(newStates: robotState) =
    Commands.runOnce({ state = newStates })

fun setShooting() = setRobotState(robotState.SHOOTING)

fun setIntakeing() = setRobotState(robotState.INTAKING)
