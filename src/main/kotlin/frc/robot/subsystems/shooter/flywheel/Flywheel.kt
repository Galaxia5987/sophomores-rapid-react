package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Flywheel : SubsystemBase(), SysIdable {
    private val velocityTorque = VelocityTorqueCurrentFOC(0.0)
    private val velocityVoltage = VoltageOut(0.0)

    private var velocitySetpoint = STOP_VELOCITY
    private val motor = UniversalTalonFX(port, config = MOTOR_CONFIG)

    val isAtSetVelocity =
        Trigger {
            mainMotor.inputs.velocity.isNear(velocitySetpoint, TOLERANCE)
        }
            .debounce(DEBOUNCE[sec])
    fun setVelocity(velocity: AngularVelocity): Command = runOnce {
        velocitySetpoint = velocity
        motor.setControl(velocityTorque.withVelocity(velocity))
    }

    fun stop() =
        setVelocity(STOP_VELOCITY).withName("FlyWheel/stop")

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystem/$name", motor.inputs)
        Logger.recordOutput("FlyWheel/IsAtSetVelocity", isAtSetVelocity)
        Logger.recordOutput("FlyWheel/SetVelocity", velocitySetpoint)
    }

    override fun setVoltage(voltage: Voltage) {
        mainMotor.setControl(VoltageOut(voltage))
    }
}
