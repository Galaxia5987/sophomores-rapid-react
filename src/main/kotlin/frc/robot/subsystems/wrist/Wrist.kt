package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.Gains
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.hood.Wrist.GEAR_RATIO
import frc.robot.subsystems.hood.Wrist.KD
import frc.robot.subsystems.hood.Wrist.KP

import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Wrist : SubsystemBase() {

    @AutoLogOutput(key = "Wrist/mechanism")
    private var mechanism = LoggedMechanism2d(4.0, 6.0)

    private var root = mechanism.getRoot("Wrist", 2.0, 1.0)

    private val ligament =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.0, 0.0))

    private val config1: TalonFXConfiguration = TalonFXConfiguration().apply {
        MotorOutput = MotorOutputConfigs().apply {
            Inverted = InvertedValue.CounterClockwise_Positive
            NeutralMode = NeutralModeValue.Brake
        }
        CurrentLimits = CurrentLimitsConfigs().apply {
            SupplyCurrentLimit = 20.0
            StatorCurrentLimit = 40.0
            SupplyCurrentLimitEnable = true
            StatorCurrentLimitEnable = true
        }
        Slot0 = Slot0Configs().apply {
            kP = KP
            kD = KD
        }
        Feedback = FeedbackConfigs().apply {
            SensorToMechanismRatio = GEAR_RATIO
        }

    }
    private val simGains = Gains(kD = 0.3, kP = 1.65)
    private val WristMotor = UniversalTalonFX(3, simGains = simGains, config = config1, gearRatio = GEAR_RATIO)
    private val positionReq1: PositionVoltage = PositionVoltage(0.0)
    private var setpoint: Angle = Degrees.of(0.0)

    fun setAngle(angle: Angle): Command {
        return Commands.runOnce({
            setpoint = angle
            WristMotor.setControl(positionReq1.withPosition(angle))
        })
    }

    fun open(): Command {
        return setAngle(25.0.degrees)
    }

    fun close(): Command {
        return setAngle(0.0.degrees)
    }

    override fun periodic() {
WristMotor.updateInputs()
        ligament.setAngle(WristMotor.inputs.position[degrees])
        Logger.processInputs("Wrist",WristMotor.inputs)
        Logger.recordOutput("Wrist/targetAngle",setpoint)
        Logger.recordOutput("Wrist/ligament",mechanism)
    }
}
