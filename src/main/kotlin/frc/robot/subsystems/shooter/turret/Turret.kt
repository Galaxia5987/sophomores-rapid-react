package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Turret : SubsystemBase() {
    val motor = UniversalTalonFX(MOTOR_PORT, config = MOTOR_CONFIG)
    var angleSetpoint = Units.Rotations.zero()
    val sensor = DigitalInput(SENSOR_PORT)
    val motionMagicTorque = MotionMagicTorqueCurrentFOC(0.0)
    val isAtResetPoint =
        Trigger { sensor.get() }
            .onTrue(runOnce { motor.reset(Units.Rotations.zero()) })
    val isAtSetpoint = Trigger { motor.inputs.position == angleSetpoint }
    fun setAngle(position: Angle) = runOnce {
        motor.setControl(motionMagicTorque.withPosition(position))
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystem/$name", motor.inputs)
        Logger.recordOutput("Turret/isAtResetPoint", isAtResetPoint)
        Logger.recordOutput("Turret/isAtSetpoint", isAtSetpoint)
    }
}
