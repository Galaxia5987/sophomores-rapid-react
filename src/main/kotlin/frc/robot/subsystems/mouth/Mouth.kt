package frc.robot.subsystems.mouth

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX

object Mouth: SubsystemBase() {
    private val motor1: UniversalTalonFX = UniversalTalonFX(0, config = config, gearRatio = ratio)
    private val voltageRequest: VoltageOut = VoltageOut(0.0)
    private var setVolts: Voltage = 0.0.volts

    private fun setVoltage(voltage: Voltage): Command {
        return Commands.runOnce({
            setVolts = voltage
            motor1.setControl(voltageRequest.withOutput(voltage))
        })
    }

    fun setActionInTake(){
            setVoltage(Action.INTAKE.voltage)
    }

    fun setActionOutTake(){
        setVoltage(Action.OUTTAKE.voltage)
    }

    fun setActionStop(){
        setVoltage(Action.STOP.voltage)
    }
}