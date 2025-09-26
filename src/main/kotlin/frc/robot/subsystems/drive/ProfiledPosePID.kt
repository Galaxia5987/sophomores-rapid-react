package frc.robot.subsystems.drive

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.LoggedNetworkGains
import frc.robot.lib.logged_output.LoggedOutputManager
import org.littletonrobotics.junction.Logger.recordOutput
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

var xController =
    ProfiledPIDController(
        xGains.kP.get(),
        xGains.kI.get(),
        xGains.kD.get(),
        linearLimits
    )
var yController =
    ProfiledPIDController(
        yGains.kP.get(),
        yGains.kI.get(),
        yGains.kD.get(),
        linearLimits
    )

var thetaController =
    ProfiledPIDController(
        thetaGains.kP.get(),
        thetaGains.kI.get(),
        thetaGains.kD.get(),
        rotationalLimits
    )
        .apply { enableContinuousInput(-Math.PI, Math.PI) }
val controllers =
    mapOf(
        "x" to { xController },
        "y" to { yController },
        "theta" to { thetaController }
    )
        .apply {
            forEach { (name, controller) ->
                LoggedOutputManager.addRunnable {
                    val controller = controller.invoke()
                    recordOutput(
                        "AutoAlignment/Errors/$name",
                        controller.positionError
                    )
                    recordOutput(
                        "AutoAlignment/Setpoint/$name",
                        controller.setpoint.position
                    )
                    recordOutput(
                        "AutoAlignment/Goal/$name",
                        controller.goal.position
                    )
                    recordOutput(
                        "AutoAlignment/AtSetpoint/$name",
                        controller.atSetpoint()
                    )
                }
            }
        }

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

@LoggedOutput(path = "AutoAlignment")
var atGoal =
    Trigger(xController::atGoal)
        .and(yController::atGoal)
        .and(thetaController::atGoal)

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
fun getSpeedSetpoint(botPose: Pose2d): () -> ChassisSpeeds = {
    ChassisSpeeds(
        xController.calculate(botPose.x),
        yController.calculate(botPose.y),
        thetaController.calculate(botPose.rotation.radians)
    )
}
