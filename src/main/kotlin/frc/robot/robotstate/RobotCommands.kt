package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.flywheel
import frc.robot.hood
import frc.robot.hopper
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rad
import frc.robot.lib.extensions.rotationFromPoint
import frc.robot.lib.extensions.rps
import frc.robot.lib.getPose2d
import frc.robot.roller
import frc.robot.subsystems.drive.alignToPose
import frc.robot.turret

val robotDistanceFromBasket
    get() = drive.pose.distanceFromPoint(HUB_LOCATION.translation)
val turretRotationToBasket
    get() = drive.pose.rotationFromPoint(HUB_LOCATION.translation)
val isOuterDeadZone
    get() =
        robotDistanceFromBasket > MAX_DISTANCE_FROM_BASKET


fun driveToShootingPoint() = drive.defer {
    val distance = if (isOuterDeadZone) MAX_DISTANCE_FROM_BASKET else MIN_DISTANCE_FROM_BASKET
    alignToPose(
        getPose2d(
            drive.pose.translation / (robotDistanceFromBasket[m] / distance[m])
        )
    )
}.withName("Drive/Drive to shooting point")

fun shoot() =
    Commands.sequence(
        drive.lock(),
        flywheel.setVelocity(0.rps),
        hopper.start(), // TODO() place Holder 0.rps
        roller.intake()
    ).withName("Drive/Shoot")

fun setDefaultCommands() {
    turret.defaultCommand = run {
        turret.setAngle(turretRotationToBasket)
    }
    hood.defaultCommand = run {
        hood.setAngle(0.rad)
    } // TODO() place Holder 0.rad
//    drive.defaultCommand = run {
//        TODO() add auto rotation to drive
//    }
}
