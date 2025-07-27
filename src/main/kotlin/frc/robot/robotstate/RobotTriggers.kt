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

var state = RobotStates.Idle
val BallsFull = roller.HasBall.and(hopper.hasBall)
val BallsEmpty = roller.HasBall.negate().and(hopper.hasBall.negate())
val IsShooting = Trigger { state == RobotStates.Shooting }
val isOuterDeadZone = Trigger {
    RobotDistanceFromBasket > MaxDistanceFromBasket
}
val isInDeadZone = Trigger {
    RobotDistanceFromBasket[m].between(
        MinDistanceFromBasket[m],
        MaxDistanceFromBasket[m]
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
        and(isInDeadZone).onTrue(Shoot())
        and(isInDeadZone.negate()).onTrue(DriveToShootingPoint())
    }
}

private fun SetRobotState(newStates: RobotStates) =
    Commands.runOnce({ state = newStates })

fun SetShooting() = SetRobotState(RobotStates.Shooting)

fun SetIntakeing() = SetRobotState(RobotStates.Intakeing)
