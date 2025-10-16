package frc.robot.subsystems.roller

import com.ctre.phoenix6.controls.VoltageOut
import com.revrobotics.ColorSensorV3
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.colorSimilarity
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.kilogramSquareMeters
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.shooter.hopper.BLUE_COLOR
import frc.robot.subsystems.shooter.hopper.RED_COLOR
import frc.robot.subsystems.shooter.hopper.SIMILARITY_THRESHOLD
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LoggedOutput

object Roller : SubsystemBase() {
    private val motor =
        UniversalTalonFX(MOTOR_PORT, momentOfInertia = 0.002.kg2m)

    private val auxiliaryMotor =
        UniversalTalonFX(
            AUXILIARY_MOTOR_PORT,
            momentOfInertia = (0.002).kilogramSquareMeters,
        )

    private val voltageRequest = VoltageOut(0.0)

    private val colorSensor = ColorSensorV3(I2C.Port.kOnboard)

    @LoggedOutput
    val ballColor
        get() = colorSensor.color
    @LoggedOutput
    val isBallRed = Trigger {
        ballColor.colorSimilarity(RED_COLOR) > SIMILARITY_THRESHOLD
    }
    @LoggedOutput
    val isBallBlue = Trigger {
        ballColor.colorSimilarity(BLUE_COLOR) > SIMILARITY_THRESHOLD
    }
    @LoggedOutput val hasBall = isBallRed.or(isBallBlue)

    private fun setVoltage(voltage: Voltage): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun intake(): Command = setVoltage(INTAKE)

    fun outtake(): Command = setVoltage(OUTTAKE)

    fun stop(): Command = setVoltage(STOP)

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
