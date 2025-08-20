package frc.robot.robotstate

import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Commands.parallel
import edu.wpi.first.wpilibj2.command.Commands.sequence
import edu.wpi.first.wpilibj2.command.Commands.waitUntil
import frc.robot.drive
import frc.robot.flywheel
import frc.robot.hopper
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.toRotation2d
import frc.robot.lib.getPose2d
import frc.robot.lib.named
import frc.robot.roller
import frc.robot.subsystems.drive.alignToPose
import frc.robot.subsystems.shooter.flywheel.SLOW_ROTATION
import frc.robot.subsystems.shooter.hood.HoodAngles
import frc.robot.subsystems.shooter.turret.MAX_ANGLE
import frc.robot.subsystems.shooter.turret.MIN_ANGLE

val robotDistanceFromHub
    get() = drive.pose.distanceFromPoint(HUB_LOCATION)

val angleToHub
    get() = drive.pose.translation.rotationToPoint(HUB_LOCATION).measure

val turretRotationToHub: Angle
    get() = angleToHub.coerceIn(MIN_ANGLE, MAX_ANGLE)

val swerveAngle
    get() = angleToHub - turretRotationToHub

val hoodAngle
    get() =
        when (robotDistanceFromHub) {
            in HoodAngles.NEAR.range -> HoodAngles.NEAR.angle
            in HoodAngles.MID.range -> HoodAngles.MID.angle
            in HoodAngles.FAR.range -> HoodAngles.FAR.angle
            else -> 45.deg
        }

val flywheelTargetVelocity
    get() =
        when (robotDistanceFromHub[m]) {
            in 0.2..0.6 -> 10
            in 0.6..1.0 -> 20
            in 1.0..1.4 -> 30
            in 1.4..1.8 -> 40
            in 1.8..2.2 -> 50
            in 2.2..2.6 -> 60
            in 2.6..3.0 -> 70
            in 3.0..3.4 -> 80
            in 3.4..3.8 -> 90
            in 3.8..4.0 -> 100
            else -> SLOW_ROTATION[rps]
        }.rps

fun driveToShootingPoint() =
    drive
        .defer {
            val robotTranslation = drive.pose.translation
            val setpoint =
                if (
                    INNER_SHOOTING_AREA.getDistance(robotTranslation) <
                        OUTER_SHOOTING_AREA.getDistance(robotTranslation)
                ) {
                    INNER_SHOOTING_AREA.nearest(robotTranslation)
                } else {
                    OUTER_SHOOTING_AREA.nearest(robotTranslation)
                }
            alignToPose((getPose2d(setpoint, swerveAngle.toRotation2d())))
        }
        .named("Drive")

fun startShooting() =
    sequence(
            drive.lock(),
            waitUntil(flywheel.isAtSetVelocity),
            parallel(hopper.start(), roller.intake())
        )
        .named(COMMAND_NAME_PREFIX)

fun stopShooting() =
    parallel(hopper.stop(), roller.stop())
        .named(COMMAND_NAME_PREFIX)

fun startIntaking() =
    parallel(roller.intake(), hopper.start())
        .named(COMMAND_NAME_PREFIX)

fun stopIntaking() =
    parallel(roller.stop(), hopper.stop())
        .named(COMMAND_NAME_PREFIX)
