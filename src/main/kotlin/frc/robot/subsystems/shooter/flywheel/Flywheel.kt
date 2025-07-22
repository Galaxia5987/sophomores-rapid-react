package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Flywheel : SubsystemBase() {
    private val velocityTorque = VelocityTorqueCurrentFOC(0.0)
    private var velocitySetpoint = STOP_VELOCITY
    private val motor = UniversalTalonFX(port, config = MOTOR_CONFIG)

    val isAtSetVelocity = Trigger { motor.inputs.velocity == velocitySetpoint }

    fun setVelocity(velocity: AngularVelocity): Command = runOnce {
        velocitySetpoint = velocity
        motor.setControl(velocityTorque.withVelocity(velocity))
    }

    fun stop() =
        setVelocity(STOP_VELOCITY)
            .alongWith(runOnce { setVelocity = STOP_VELOCITY })

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("flyWheelInputs", motor.inputs)
        Logger.recordOutput("flyWheel/isAtSetVelocity", isAtSetVelocity)
    }
}
