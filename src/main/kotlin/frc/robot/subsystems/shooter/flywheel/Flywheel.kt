package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

class Flywheel : SubsystemBase() {
    private val velocityTorque = VelocityTorqueCurrentFOC(0.0)
    private var setVelocity = STOP_VELOCITY
    private val motor = UniversalTalonFX(port, config = MOTOR_CONFIG)

    val isAtSetVelocity = Trigger { motor.inputs.velocity == setVelocity }

    private fun velocity(velocity: AngularVelocity) {
        setVelocity = velocity
        motor.setControl(velocityTorque.withVelocity(velocity))
    }
    fun stop() {
        setVelocity = STOP_VELOCITY
        motor.setControl(velocityTorque.withVelocity(STOP_VELOCITY))
    }
    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("flyWheelInputs", motor.inputs)
        Logger.recordOutput("flyWheel/isAtSetVelocity",isAtSetVelocity)
    }
}
