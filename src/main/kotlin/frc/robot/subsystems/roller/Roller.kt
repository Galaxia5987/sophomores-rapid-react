package frc.robot.subsystems.roller

import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.kilogramSquareMeters
import frc.robot.lib.extensions.volts
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput

val MOTOR_PORT = 0
val SENSOR_ID = 0

class Roller : SubsystemBase() {
    private val motor =
        UniversalTalonFX(
            MOTOR_PORT,
            momentOfInertia = (0.002).kilogramSquareMeters,
        )

    private val voltageRequest = VoltageOut(0.0)

    private val rangeSensor =
        UnifiedCANRange(
            subsystemName = "Roller",
            port = SENSOR_ID,
            canbus = "",
            configuration = CANrangeConfiguration()
        )

    @AutoLogOutput
    var hasFrontBall: Boolean = false
        private set

    val HasBall = Trigger { hasFrontBall }

    private fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun intake(): Command = runOnce { setVoltage(3.0.volts) }

    fun outtake(): Command = runOnce { setVoltage((-3.0).volts) }

    fun stop(): Command = runOnce { setVoltage(0.0.volts) }

    override fun periodic() {
        motor.updateInputs()
        rangeSensor.updateInputs()

        hasFrontBall = rangeSensor.isInRange
    }
}
