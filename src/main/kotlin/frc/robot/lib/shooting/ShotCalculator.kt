package frc.robot.lib.shooting

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.rotationToPoint
import frc.robot.robotstate.HUB_LOCATION
import kotlin.math.hypot

data class ShotData(
    val compensatedTarget: Translation2d,
    val turretAngle: Rotation2d,
    val compensatedDistance: Distance
)

fun calculateShot(
    robotPose: Pose2d,
    hubPose: Translation2d,
    speeds: ChassisSpeeds,
    shooterExitVelocity: LinearVelocity
): ShotData {
    val robotToHub = hubPose.minus(robotPose.translation)
    val distance = hypot(robotToHub.x, robotToHub.y)
    val shooterSpeed = shooterExitVelocity[mps]
    val velocityVector =
        Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)
    val velocityNorm = hypot(velocityVector.x, velocityVector.y)

    if (
        MathUtil.isNear(0.0, shooterSpeed, 0.15) ||
            MathUtil.isNear(0.0, velocityNorm, 0.15)
    ) {
        // No motion compensation, just regular interpolation
        val turretAngle =
            robotPose.translation.rotationToPoint(robotToHub) -
                robotPose.rotation
        return ShotData(
            compensatedTarget = HUB_LOCATION,
            turretAngle = turretAngle,
            compensatedDistance = distance.m
        )
    }

    // On move compensation
    val shotTime = distance / shooterSpeed
    val shotOffset = velocityVector.times(shotTime)
    val compensatedTarget =
        hubPose.plus(shotOffset).minus(robotPose.translation)
    val turretAngle =
        robotPose.translation.rotationToPoint(compensatedTarget) -
            robotPose.rotation
    val compensatedDistance =
        robotPose.translation.getDistance(compensatedTarget).m

    return ShotData(compensatedTarget, turretAngle, compensatedDistance)
}
