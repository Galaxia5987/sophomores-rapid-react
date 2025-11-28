package frc.robot.subsystems.hopper

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.hood.Hood.mechanism
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.team5987.annotation.LoggedOutput

object Hopper : SubsystemBase() {
    @LoggedOutput var machanism = LoggedMechanism2d(4.0, 60.0)
    private var root = mechanism.getRoot("Hopper", 2.0, 1.0)
    private val motor =
        UniversalTalonFX(port = port, simGains = simGains, config = counfig)
    @LoggedOutput var voltageReq: VoltageOut = VoltageOut(0.0)

    fun setVoltage(voltage: Voltage) = runOnce {
        motor.setControl(voltageReq.withOutput(voltage))
    }

    fun intake(): Command {
        return setVoltage(10.volts)
    }
    fun outTake(): Command {
        return setVoltage((-10).volts)
    }
    fun stop(): Command {
        return setVoltage(0.volts)
    }

    override fun periodic() {
        motor.updateInputs()
        setVoltage(motor.inputs.voltage)
        Logger.processInputs("Hopper", motor.inputs)
    }
}
