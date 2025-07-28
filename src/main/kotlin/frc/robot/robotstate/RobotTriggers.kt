package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.flywheel
import frc.robot.hopper
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.roller

private val ballsEmpty = roller.HasBall.negate().and(hopper.hasBall.negate())
private val IsShooting = Trigger { state == ROBOT_STATE.Shooting }
val isOuterDeadZone = Trigger {
    robotDistanceFromBasket > MAX_DISTANCE_FROM_BASKET
}
private val isInDeadZone = Trigger {
    robotDistanceFromBasket[m] in
            MIN_DISTANCE_FROM_BASKET[m]..
            MAX_DISTANCE_FROM_BASKET[m]
}

fun bindRobotStateTriggers() {
    IsShooting.apply {
        and(ballsEmpty).apply {
            onTrue(
                parallel(
                    setIntakeing(),
                    flywheel.slowRotation(),
                    hopper.stop(),
                    roller.stop()
                )
            ) // TODO() place Holder 0.rps
        }
        and(isInDeadZone.negate()).onTrue(shoot())
        and(isInDeadZone).onTrue(driveToShootingPoint())
    }
}

private fun setRobotState(newStates: ROBOT_STATE) =
    Commands.runOnce({ state = newStates })

fun setShooting() = setRobotState(ROBOT_STATE.Shooting)

fun setIntakeing() = setRobotState(ROBOT_STATE.Intaking)
