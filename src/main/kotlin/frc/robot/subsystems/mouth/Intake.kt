package frc.robot.subsystems.mouth

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LoggedOutput

object Intake : SubsystemBase() {
    @LoggedOutput private val motor: UniversalTalonFX = UniversalTalonFX(0, config = config, gearRatio = RATIO)
    @LoggedOutput private val voltageRequest: VoltageOut = VoltageOut(0.0)
    @LoggedOutput private var setpoint: Voltage = 0.volts

    private fun setVoltage(voltage: Voltage) = runOnce {
        setpoint = voltage
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun intake() = runOnce { setVoltage(10.volts) }

    fun outtake() = runOnce { setVoltage((-10).volts) }

    fun stop() = runOnce { setVoltage(0.volts) }
}
