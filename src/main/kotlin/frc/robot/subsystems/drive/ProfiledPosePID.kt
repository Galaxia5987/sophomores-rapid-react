package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.LoggedNetworkGains
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LoggedOutput

private val gainsX = LoggedNetworkGains("gainX", 4.0)
private val gainsY =
    LoggedNetworkGains(
        "gainY",
    )
private val gainsTheta = LoggedNetworkGains("gainTheta", 6.0)

private val LINEAR_CONSTRAINTS =
    Constraints(
        TunerConstants.kSpeedAt12Volts.`in`(Units.MetersPerSecond),
        Units.MetersPerSecondPerSecond.of(2.8)
            .`in`(Units.MetersPerSecondPerSecond)
    )

private val ROTATIONAL_CONSTRAINTS =
    Constraints(
        TunerConstants.kMaxOmegaVelocity.`in`(Units.RadiansPerSecond),
        Units.DegreesPerSecondPerSecond.of(360.0)
            .`in`(Units.RadiansPerSecondPerSecond)
    )

val xController =
    ProfiledPIDController(
        gainsX.kP.get(),
        gainsX.kI.get(),
        gainsX.kD.get(),
        LINEAR_CONSTRAINTS
    )
val yController =
    ProfiledPIDController(
        gainsY.kP.get(),
        gainsY.kI.get(),
        gainsY.kD.get(),
        LINEAR_CONSTRAINTS
    )

val thetaController =
    ProfiledPIDController(
            gainsTheta.kP.get(),
            gainsTheta.kI.get(),
            gainsTheta.kD.get(),
            ROTATIONAL_CONSTRAINTS
        )
        .apply { enableContinuousInput(-Math.PI, Math.PI) }

fun updateProfiledPID() {
    xController.setPID(gainsX.kP.get(), gainsX.kI.get(), gainsX.kD.get())
    yController.setPID(gainsY.kP.get(), gainsY.kI.get(), gainsY.kD.get())
    thetaController.setPID(
        gainsTheta.kP.get(),
        gainsTheta.kI.get(),
        gainsTheta.kD.get()
    )
}

fun setGoal(desiredPose: Pose2d) {
    updateProfiledPID()
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setGoal(desiredPose.rotation.radians)
}

val atGoal: Trigger =
    Trigger(xController::atGoal)
        .and(yController::atGoal)
        .and(thetaController::atGoal)
        .debounce(0.05)

fun resetProfiledPID(botPose: Pose2d, botSpeeds: ChassisSpeeds) {
    xController.reset(botPose.x, botSpeeds.vxMetersPerSecond)
    yController.reset(botPose.y, botSpeeds.vyMetersPerSecond)
    thetaController.reset(
        botPose.rotation.radians,
        botSpeeds.omegaRadiansPerSecond
    )
}

fun setTolerance(pose2d: Pose2d) {
    xController.setTolerance(pose2d.x)
    yController.setTolerance(pose2d.y)
    thetaController.setTolerance(pose2d.rotation.radians)
}

/**
 * Returns field relative chassis speeds to the selected goal.
 * @botPose the current pose of the robot
 */
@LoggedOutput
object AutoAlignment {
    var XError = xController.positionError
    var YError = yController.positionError
    var ThetaError = thetaController.positionError
    var XSetpoint = xController.setpoint.position
    var YSetpoint = yController.setpoint.position
    var ThetaSetpoint = thetaController.setpoint.position
    var XGoal = xController.goal.position
    var YGoal = yController.goal.position
    var ThetaGoal = thetaController.goal.position
    var AtGoal = atGoal.asBoolean
    var XAtSetpoint = xController.atSetpoint()
    var YAtSetpoint = yController.atSetpoint()
    var thetaAtSetpoint = thetaController.atSetpoint()
    var GoalPose =
        Pose2d(
            xController.goal.position,
            yController.goal.position,
            Rotation2d.fromRadians(thetaController.goal.position)
        )
}

fun getSpeed(botPose: Pose2d): () -> ChassisSpeeds = {
    val fieldRelativeSpeeds =
        ChassisSpeeds(
            xController.calculate(botPose.x),
            yController.calculate(botPose.y),
            thetaController.calculate(botPose.rotation.radians)
        )
    Logger.recordOutput(
        "AutoAlignment/fieldRelativeSpeeds",
        fieldRelativeSpeeds
    )
    fieldRelativeSpeeds
}
