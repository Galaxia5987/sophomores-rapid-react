package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.log
import frc.robot.lib.getPose3d
import frc.robot.lib.getRotation3d
import frc.robot.subsystems.drive.Drive
import org.littletonrobotics.junction.Logger

//private val swerveModulePose: Array<Translation2d> =
//    Drive.getModuleTranslations()
//
//private fun getSwerveModulePoseTurn(
//    moduleX: Double,
//    moduleY: Double,
//    moduleYaw: Angle
//): Pose3d {
//    return Pose3d(
//        Translation3d(moduleX, moduleY, kWheelRadius.`in`(Meters)),
//        getRotation3d(yaw = moduleYaw)
//    )
//}
//
//private fun getSwerveModulePoseDrive(
//    moduleX: Double,
//    moduleY: Double,
//    moduleYaw: Angle,
//    modulePitch: Angle
//): Pose3d {
//
//    return Pose3d(
//        Translation3d(moduleX, moduleY, kWheelRadius.`in`(Meters)),
//        getRotation3d(yaw = moduleYaw, pitch = modulePitch)
//    )
//}
//
//private fun getAllSwerveModulePoseTurn(): Array<Pose3d> {
//    val swervePosesTurn: Array<Pose3d> =
//        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
//    for (i in 0..3) {
//        swervePosesTurn[i] =
//            getSwerveModulePoseTurn(
//                swerveModulePose[i].x,
//                swerveModulePose[i].y,
//                driveSimulation!!.modules[i].driveWheelFinalPosition
//            )
//    }
//    return swervePosesTurn
//}
//
//private fun getAllSwerveModulePoseDrive(): Array<Pose3d> {
//    val swervePosesDrive: Array<Pose3d> =
//        arrayOf(Pose3d(), Pose3d(), Pose3d(), Pose3d())
//
//    for (i in 0..3) {
//        swervePosesDrive[i] =
//            getSwerveModulePoseDrive(
//                swerveModulePose[i].x,
//                swerveModulePose[i].y,
//                swerveDrive.SwerveTurnAngle[i],
//                swerveDrive.SwerveDriveAngle[i]
//            )
//    }
//    return swervePosesDrive
//}


fun getSubsystemPose(): Array<Pose3d>{
    return Array(13){getPose3d()}
}
fun logSubsystemPose(){
    Logger.recordOutput("RobotPose3d", getSubsystemPose())
}