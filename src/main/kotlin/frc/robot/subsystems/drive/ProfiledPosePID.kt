package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.units.measure.Time

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.LoggedNetworkGains
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import org.team5987.annotation.LoggedOutput

const val COMMAND_NAME_PREFIX = "AutoAlignment"

private val xGains = LoggedNetworkGains("xGains", 4.0)
private val yGains =
    LoggedNetworkGains(
        "yGains",
    )

private val thetaGains = LoggedNetworkGains("thetaGains", 6.0)
private val linearMaxVelocity =
    LoggedNetworkNumber("$COMMAND_NAME_PREFIX/linearMaxVelocity", 4.69)
private val linearMaxAcceleration =
    LoggedNetworkNumber("$COMMAND_NAME_PREFIX/linearMaxAcceleration", 2.8)

private var rotationalMaxVelocity =
    LoggedNetworkNumber("$COMMAND_NAME_PREFIX/rotationMaxVelocity", 7.0)
private var rotationalMaxAcceleration =
    LoggedNetworkNumber("$COMMAND_NAME_PREFIX/rotationMaxAcceleration", 360.0)

private val linearLimits
    get() = Constraints(linearMaxVelocity.get(), linearMaxAcceleration.get())

private val rotationalLimits
    get() =
        Constraints(
            rotationalMaxVelocity.get(),
            rotationalMaxAcceleration.get()
        )

val xController
    get() =
        ProfiledPIDController(
            xGains.kP.get(),
            xGains.kI.get(),
            xGains.kD.get(),
            linearLimits
        )
val yController
    get() =
        ProfiledPIDController(
            yGains.kP.get(),
            yGains.kI.get(),
            yGains.kD.get(),
            linearLimits
        )

val thetaController
    get() =
        ProfiledPIDController(
                thetaGains.kP.get(),
                thetaGains.kI.get(),
                thetaGains.kD.get(),
                rotationalLimits
            )
            .apply { enableContinuousInput(-Math.PI, Math.PI) }

fun updateProfiledPIDGains() {
    mapOf(
            xController to xGains,
            yController to yGains,
            thetaController to thetaGains
        )
        .forEach { (controller, gains) ->
            controller.setPID(gains.kP.get(), gains.kI.get(), gains.kD.get())
        }
}

fun setGoal(desiredPose: Pose2d) {
    updateProfiledPIDGains()
    xController.setGoal(desiredPose.x)
    yController.setGoal(desiredPose.y)
    thetaController.setGoal(desiredPose.rotation.radians)
}

lateinit var atGoal: Trigger

fun initAtSetGoalTrigger(debounce: Time) {
    atGoal =
        Trigger(xController::atGoal)
            .and(yController::atGoal)
            .and(thetaController::atGoal)
            .debounce(debounce[sec])
}

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
    var xError = xController.positionError
    var yError = yController.positionError
    var thetaError = thetaController.positionError
    var xSetpoint = xController.setpoint.position
    var ySetpoint = yController.setpoint.position
    var thetaSetpoint = thetaController.setpoint.position
    var xGoal = xController.goal.position
    var yGoal = yController.goal.position
    var thetaGoal = thetaController.goal.position
    var atGoalSupplier = atGoal.asBoolean
    var xAtSetpoint = xController.atSetpoint()
    var yAtSetpoint = yController.atSetpoint()
    var thetaAtSetpoint = thetaController.atSetpoint()
}

fun getSpeedSetpoint(botPose: Pose2d): () -> ChassisSpeeds = {
    ChassisSpeeds(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians)
    )
}
