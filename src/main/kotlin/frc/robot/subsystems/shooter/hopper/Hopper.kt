package frc.robot.subsystems.shooter.hopper

import com.ctre.phoenix6.controls.VoltageOut
import com.revrobotics.ColorSensorV3
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.CURRENT_MODE
import frc.robot.Mode
import frc.robot.lib.colorSimilarity
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.team5987.annotation.LoggedOutput

object Hopper : SubsystemBase() {

    private val motor: UniversalTalonFX =
        UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)

    private val colorSensor = ColorSensorV3(I2C_PORT)
    private val voltageRequest = VoltageOut(0.0)

    @LoggedOutput
    val ballColor
        get() = colorSensor.color

    @LoggedOutput
    val redConfidence
        get() = ballColor.colorSimilarity(RED_COLOR)

    @LoggedOutput
    val blueConfidence
        get() = ballColor.colorSimilarity(BLUE_COLOR)

    @LoggedOutput
    val isBallRed = Trigger { redConfidence > SIMILARITY_THRESHOLD }

    @LoggedOutput
    val isBallBlue = Trigger { blueConfidence > SIMILARITY_THRESHOLD }

    val simulatedHasBall = LoggedNetworkBoolean("/Tuning/Hopper/hasBall", false)

    @LoggedOutput
    val hasBall =
        if (CURRENT_MODE == Mode.REAL) isBallBlue.or(isBallRed)
        else Trigger { simulatedHasBall.get() }

    private fun setVoltage(voltage: Voltage): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun start(): Command = setVoltage(INTAKE_VOLTAGE)

    fun stop(): Command = setVoltage(0.volts)

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
