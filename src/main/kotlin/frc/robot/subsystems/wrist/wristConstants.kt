package frc.robot.subsystems.Wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains

val GEAR_RATIO=1/69.82
val REAL_GAINS: Gains= Gains(1.7 , kD = 0.0)
val port =3
val simGains = Gains(kD = 0.3, kP = 1.65)
 val config: TalonFXConfiguration = TalonFXConfiguration().apply {
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
        kP = REAL_GAINS.kP
        kD = REAL_GAINS.kD
    }
    Feedback = FeedbackConfigs().apply {
        SensorToMechanismRatio = GEAR_RATIO
    }}