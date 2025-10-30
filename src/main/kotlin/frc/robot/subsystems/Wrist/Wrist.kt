package frc.robot.subsystems.hood.Wrist

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
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.Gains
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Wrist : SubsystemBase(){

    @AutoLogOutput(key = "Wrist/mechanism")
    private var mechanism = LoggedMechanism2d(4.0, 6.0)

    private var root = mechanism.getRoot("Wrist", 2.0, 1.0)

    private val ligament =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.0, 0.0))

 private val   config1: TalonFXConfiguration = TalonFXConfiguration().apply {
     MotorOutput= MotorOutputConfigs().apply {
         Inverted= InvertedValue.CounterClockwise_Positive
         NeutralMode= NeutralModeValue.Brake
     }
     CurrentLimits= CurrentLimitsConfigs().apply{
         SupplyCurrentLimit=20.0
         StatorCurrentLimit=40.0
         SupplyCurrentLimitEnable=true
         StatorCurrentLimitEnable=true
     }
     Slot0= Slot0Configs().apply {
        kP=1.7
        kD=0.0
     }
     Feedback= FeedbackConfigs().apply{
         SensorToMechanismRatio = GEAR_RATIO3
     }

 }
 private val simGains= Gains(kD = 0.22, kP = 1.65)
    private val WristMotor= UniversalTalonFX(3)
   private val positionReq1: PositionVoltage= PositionVoltage(0.0)
    private val setPoint : Angle= Degrees.of(0.0)

    fun openWrist(): Command
}