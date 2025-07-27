package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.flywheel
import frc.robot.hopper
import frc.robot.lib.between
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.roller

private var state = ROBOT_STATE.Idle
private val BallsEmpty = roller.HasBall.negate().and(hopper.hasBall.negate())
private val IsShooting = Trigger { state == ROBOT_STATE.Shooting }
val isOuterDeadZone = Trigger {
    robotDistanceFromBasket > MAX_DISTANCE_FROM_BASKET
}
private val isInDeadZone = Trigger {
    robotDistanceFromBasket[m].between(
        MIN_DISTANCE_FROM_BASKET[m],
        MAX_DISTANCE_FROM_BASKET[m]
    )
}

fun bindRobotStateTriggers() {
    IsShooting.apply {
        and(BallsEmpty).apply {
            onTrue(SetIntakeing())
            onTrue(flywheel.setVelocity(0.rps)) // TODO() place Holder 0.rps
            onTrue(hopper.stop())
            onTrue(roller.stop())
        }
        and(isInDeadZone).onTrue(shoot())
        and(isInDeadZone.negate()).onTrue(driveToShootingPoint())
    }
}

private fun SetRobotState(newStates: ROBOT_STATE) =
    Commands.runOnce({ state = newStates })

fun SetShooting() = SetRobotState(ROBOT_STATE.Shooting)

fun SetIntakeing() = SetRobotState(ROBOT_STATE.Intakeing)
