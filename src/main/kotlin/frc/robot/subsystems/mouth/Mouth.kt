package frc.robot.subsystems.mouth

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Mouth : SubsystemBase() {
    private val motor: UniversalTalonFX = UniversalTalonFX(0, config = config, gearRatio = RATIO)
    private val voltageRequest: VoltageOut = VoltageOut(0.0)
    private var targetVoltage: Voltage = 0.0.volts

    private fun setVoltage(voltage: Voltage): Command {
        return Commands.runOnce({
            targetVoltage = voltage
            motor.setControl(voltageRequest.withOutput(voltage))
        })
    }

    fun setActionInTake(): Command {
        return setVoltage(10.0.volts)
    }

    fun setActionOutTake(): Command {
        return setVoltage((-10.0).volts)
    }

    fun setActionStop(): Command {
        return setVoltage(0.0.volts)
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Mouth", motor.inputs)
        Logger.recordOutput("Mouth/targetVoltage", targetVoltage)
    }
}
