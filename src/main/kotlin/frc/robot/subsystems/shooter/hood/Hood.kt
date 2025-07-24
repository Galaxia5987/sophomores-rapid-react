package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.CURRENT_MODE
import frc.robot.Mode
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

class Hood : SubsystemBase() {

    private val motor = UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)

    private val positionRequest = PositionTorqueCurrentFOC(0.0)

    private val encoder = CANcoder(ENCODER_ID)

    private var setpoint: Angle = 0.deg

    private val isAtSetpoint = Trigger {
        setpoint.isNear(motor.inputs.position, SETPOINT_TOLERANCE)
    }

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    fun setAngle(angle: Angle): Command = runOnce {
        setpoint = angle
        motor.setControl(positionRequest.withPosition(angle))
    }

    override fun periodic() {
        motor.updateInputs()
        if (CURRENT_MODE == Mode.REAL) {
            motor.reset(encoder.absolutePosition.value)
        }
        Logger.processInputs("Subsystems/$name", motor.inputs)
        Logger.recordOutput("Subsystems/$name/isAtSetpoint", isAtSetpoint)
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
    }
}
