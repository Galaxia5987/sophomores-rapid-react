package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.kilogramSquareMeters
import frc.robot.lib.universal_motor.UniversalTalonFX
import kotlin.math.absoluteValue
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger

private const val GEAR_RATIO = 0.0
val MOTOR_PORT = 0

class Wrist : SubsystemBase() {
    @AutoLogOutput private var setpoint: Angle = 0.0.degrees

    private val motor =
        UniversalTalonFX(
            MOTOR_PORT,
            momentOfInertia = (0.0025).kilogramSquareMeters,
            gearRatio = GEAR_RATIO,
            config = TalonFXConfiguration()
        )

    private val positionRequest = PositionVoltage(0.0)

    fun setAngle(angle: Angle): Command = runOnce {
        setpoint = angle
        motor.setControl(positionRequest.withPosition(angle))
    }


    fun reset(): Command = runOnce {
        setpoint = 0.0.degrees
        motor.setControl(positionRequest.withPosition(setpoint))
    }

    fun isNearSetpoint(): Boolean {
        val error = motor.inputs.position.minus(setpoint).`in`(degrees)
        return error.absoluteValue < 1.0
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Wrist", motor.inputs)
    }
}
