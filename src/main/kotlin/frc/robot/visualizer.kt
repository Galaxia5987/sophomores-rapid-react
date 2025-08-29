package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.lib.getTranslation3d
import frc.robot.subsystems.drive.Drive
import org.littletonrobotics.junction.Logger

private val swerveModulePose: Array<Translation2d> =
    Drive.getModuleTranslations()

private val kWheelRadius = 0.0508.m

private fun getSwerveModulePoseTurn(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle
): Pose3d {
    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw)
    )
}

private fun getSwerveModulePoseDrive(
    moduleX: Double,
    moduleY: Double,
    moduleYaw: Angle,
    modulePitch: Angle
): Pose3d {

    return Pose3d(
        Translation3d(moduleX, moduleY, kWheelRadius[m]),
        getRotation3d(yaw = moduleYaw, pitch = modulePitch)
    )
}

private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
    val swervePosesTurn: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
    for (i in 0..3) {
        swervePosesTurn[i] =
            getSwerveModulePoseTurn(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i]
            )
    }
    return swervePosesTurn
}

private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
    val swervePosesDrive: Array<Pose3d> =
        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())

    for (i in 0..3) {
        swervePosesDrive[i] =
            getSwerveModulePoseDrive(
                swerveModulePose[i].x,
                swerveModulePose[i].y,
                drive.SwerveTurnAngle[i],
                drive.SwerveDriveAngle[i]
            )
    }
    return swervePosesDrive
}

val wristTranslation
    get() = getTranslation3d(0.0,0.0,0.3)
val wristRotation
    get() = getRotation3d(pitch = wrist.inputs.position)
val wristPose
    get() = getPose3d(wristTranslation, wristRotation)

val turretTranslation
    get() = getTranslation3d(0.0)
val turretRotation
    get() = getRotation3d(yaw = turret.input.position)
val turretPose
    get() = getPose3d(turretTranslation, turretRotation)

val hoodTranslation
    get() = getTranslation3d(0.0)
val hoodRotation
    get() = turretRotation + getRotation3d(pitch = hood.inputs.position)
val hoodPose
    get() = getPose3d(hoodTranslation, hoodRotation)

val flywheelTranslation
    get() = getTranslation3d(0.0)
val flywheelRotation
    get() = hoodRotation + getRotation3d(pitch = (-10).deg)
val flywheelPose
    get() = getPose3d(flywheelTranslation, flywheelRotation)

val subsystemPoseArray = Array(19) { getPose3d() }

fun getSubsystemPose(): Array<Pose3d> {
    val swerveModulesPoses = getAllSwerveModulePoseDrive()

    swerveModulesPoses.forEachIndexed { i, modulePose ->
        subsystemPoseArray[2*i+1] = modulePose
    }
    subsystemPoseArray[8] = wristPose
    subsystemPoseArray[5] = turretPose
    subsystemPoseArray[6] = hoodPose
    subsystemPoseArray[7] = flywheelPose

    return subsystemPoseArray
}

fun logSubsystemPose() {
    Logger.recordOutput("RobotPose3d", *getSubsystemPose())
}
