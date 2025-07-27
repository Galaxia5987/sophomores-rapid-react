package frc.robot.robotstate

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
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
    get() = drive.pose.distanceFromPoint(BASKET_LOCATION.translation)
val turretRotationToBasket
    get() = drive.pose.rotationFromPoint(BASKET_LOCATION.translation)

fun poseToMoveTo(distance: Distance) =
    getPose2d(
        drive.pose.translation / (robotDistanceFromBasket[m] / distance[m])
    )

fun driveToShootingPoint() =
    ConditionalCommand(
        drive.defer { alignToPose(poseToMoveTo(MAX_DISTANCE_FROM_BASKET)) },
        drive.defer { alignToPose(poseToMoveTo(MIN_DISTANCE_FROM_BASKET)) },
        isOuterDeadZone
    ).withName("RobotCommands/driveToShootingPoint")

fun shoot() =
    Commands.sequence(
        drive.lock(),
        flywheel.setVelocity(0.rps),
        hopper.start(), // TODO() place Holder 0.rps
        roller.intake()
    ).withName("RobotCommands/shoot")

fun setDefaultCommands() {
    turret.defaultCommand = run {
        turret.setAngle(turretRotationToBasket)
    }
    hood.defaultCommand = run {
        hood.setAngle(0.rad)
    } // TODO() place Holder 0.rad
}
