package frc.robot.subsystems.intake

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.Gains
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LoggedOutput

@AutoLogOutput(key = "Intake/mechanism")
var mechanism = LoggedMechanism2d(5.0, 5.0)
private var root = mechanism.getRoot("Intake", 2.5, 2.5)
val ligament =
    root.append(LoggedMechanismLigament2d("IntakeLigament", 1.0, 0.0))

object Intake : SubsystemBase() {
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            0,
            config = config,
            gearRatio = RATIO,
            simGains = Gains(kP = 1.0, kD = 0.0)
        )
    private val voltageRequest: VoltageOut = VoltageOut(0.0)
    @LoggedOutput var setpoint: Voltage = 0.volts

    private fun setVoltage(voltage: Voltage) = runOnce {
        setpoint = voltage
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun intake() = setVoltage(10.volts) // Intakes Ball

    fun outtake() = setVoltage((-10).volts) // Outtakes Ball

    fun stop() = setVoltage(0.volts) // Stops Moving

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(setpoint[volts])
        Logger.recordOutput("Intake", mechanism)
        Logger.processInputs("Subsystems/Intake", motor.inputs)
    }
}
