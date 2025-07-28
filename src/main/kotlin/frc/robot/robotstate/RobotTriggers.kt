package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.flywheel
import frc.robot.hopper
import frc.robot.lib.onTrue
import frc.robot.roller

private val ballsEmpty = roller.HasBall.negate().and(hopper.hasBall.negate())
private val IsShooting = Trigger { state == robotState.SHOOTING }
private val isInDeadZone = Trigger {
    robotDistanceFromBasket in
            MIN_DISTANCE_FROM_BASKET..
            MAX_DISTANCE_FROM_BASKET
}

fun bindRobotStateTriggers() {
    IsShooting.apply {
        and(ballsEmpty).apply {
            onTrue(
                    setIntakeing(),
                    flywheel.slowRotation(),
                    hopper.stop(),
                    roller.stop()
            ) // TODO() place Holder 0.rps
        }
        and(isInDeadZone.negate()).onTrue(shoot())
        and(isInDeadZone).onTrue(driveToShootingPoint())
    }
}

private fun setRobotState(newStates: robotState) =
    Commands.runOnce({ state = newStates })

fun setShooting() = setRobotState(robotState.SHOOTING)

fun setIntakeing() = setRobotState(robotState.INTAKING)
