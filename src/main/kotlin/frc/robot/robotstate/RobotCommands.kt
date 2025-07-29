package frc.robot.robotstate

import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.sequence
import frc.robot.drive
import frc.robot.flywheel
import frc.robot.hood
import frc.robot.hopper
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rotationFromPoint
import frc.robot.lib.extensions.rps
import frc.robot.lib.getPose2d
import frc.robot.roller
import frc.robot.subsystems.drive.alignToPose
import frc.robot.subsystems.shooter.hood.HoodAngles
import frc.robot.turret

val robotDistanceFromBasket
    get() = drive.pose.distanceFromPoint(HUB_LOCATION.translation)
val turretRotationToBasket
    get() = drive.pose.rotationFromPoint(HUB_LOCATION.translation)
val hoodAngle
    get() =
        when (robotDistanceFromBasket) {
            0.m..HoodAngles.NEAR.distance -> HoodAngles.NEAR.angles
            HoodAngles.NEAR.distance..HoodAngles.MED.distance ->
                HoodAngles.MED.angles
            HoodAngles.MED.distance..HoodAngles.FAR.distance ->
                HoodAngles.FAR.angles
            else -> 45.deg
        }
val flywheelTargetVelocity
    get() =
        when (robotDistanceFromBasket) {
            0.2..0.6 -> 1
            0.6..1.0 -> 2
            1.0..1.4 -> 3
            1.4..1.8 -> 4
            1.8..2.2 -> 5
            2.2..2.6 -> 6
            2.6..3.0 -> 7
            3.0..3.4 -> 8
            3.4..3.8 -> 9
            3.8..4.0 -> 10
            else -> {
                SLOW_ROTATION[rps]
            }
        }.rps

val isOuterDeadZone
    get() = robotDistanceFromBasket > MAX_DISTANCE_FROM_BASKET

fun driveToShootingPoint() =
    drive
        .defer {
            val distance =
                if (isOuterDeadZone) MAX_DISTANCE_FROM_BASKET
                else MIN_DISTANCE_FROM_BASKET
            alignToPose(
                getPose2d(
                    drive.pose.translation /
                            (robotDistanceFromBasket[m] / distance[m])
                )
                    .plus(HUB_LOCATION.toTransform())
            )
        }
        .withName("Drive/Drive to shooting point")

fun shooting() =
    sequence(
            drive.lock(),
            flywheel.setVelocity(0.rps),
            hopper.start(), // TODO() place Holder 0.rps
            roller.intake()
        )
        .withName("$name/Shooting")

fun stopShooting() =
    parallel(flywheel.slowRotation(), hopper.stop(), roller.stop())
        .withName("$name/StopShooting")

fun intake() =
    parallel(roller.intake(), hopper.start()).withName("$name/Intake")

fun stopIntake() =
    parallel(roller.stop(), hopper.stop()).withName("$name/StopIntake")

fun setDefaultCommands() {
    turret.defaultCommand = run { turret.setAngle(turretRotationToBasket) }
    hood.defaultCommand = run { hood.setAngle(hoodAngle) }
}
