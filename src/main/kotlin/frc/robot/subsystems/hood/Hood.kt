package frc.robot.subsystems.hood

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
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Hood : SubsystemBase() {

    @AutoLogOutput(key = "Hood/mechanism")
    private var mechanism = LoggedMechanism2d(4.0, 6.0)

    private var root = mechanism.getRoot("Hood", 2.0, 1.0)

    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))

    private val motor = UniversalTalonFX(1)
    private val positionReq: PositionVoltage = PositionVoltage(0.0)
    private var setPoint: Angle = Degrees.of(0.0)
    val config1: TalonFXConfiguration =
        TalonFXConfiguration().apply {
            MotorOutput =
                MotorOutputConfigs().apply {
                    Inverted = InvertedValue.CounterClockwise_Positive
                    NeutralMode = NeutralModeValue.Brake
                }
            CurrentLimits =
                CurrentLimitsConfigs().apply {
                    StatorCurrentLimitEnable = true
                    SupplyCurrentLimitEnable = true
                    StatorCurrentLimit = 20.0
                    SupplyCurrentLimit = 10.0
                }
            Slot0 =
                Slot0Configs().apply {
                    kP = 0.0
                    kD = 0.0
                }
            Feedback =
                FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO2 }
        }

    fun setAngle(angle: Angle): Command {
        return Commands.runOnce({
            setPoint = angle
            motor.setControl(positionReq.withPosition(angle))
        })
    }

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(motor.inputs.position[degrees])
        Logger.processInputs("Hood", motor.inputs)
        Logger.recordOutput("Hood/setAngle", setPoint)
    }
}
