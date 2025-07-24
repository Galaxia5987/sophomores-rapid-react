package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rot
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Turret : SubsystemBase() {
    private val motor = UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)
    private var angleSetpoint = 0.deg
    private val hallEffectSensor = DigitalInput(HALL_EFFECT_SENSOR_PORT)
    private val motionMagicTorque = MotionMagicTorqueCurrentFOC(0.0)
    val isAtResetPoint =
        Trigger(hallEffectSensor::get).onTrue(runOnce { motor.reset(0.rot) })
    val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(angleSetpoint, TOLERANCE)
    }
    fun setAngle(position: Angle) = runOnce {
        motor.setControl(motionMagicTorque.withPosition(position))
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
        Logger.recordOutput("Turret/isAtResetPoint", isAtResetPoint)
        Logger.recordOutput("Turret/isAtSetpoint", isAtSetpoint)
    }
}
