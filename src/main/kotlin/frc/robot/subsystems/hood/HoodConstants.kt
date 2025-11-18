package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains

val port = 0
val simGains = Gains(kP = 1.3, kD = 0.25)
val GEAR_RATIO = 59.5

val REAL_GAINS: Gains = Gains(kP = 1.5, kD = 0.1)
val config: TalonFXConfiguration =
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
                kP = REAL_GAINS.kP
                kD = REAL_GAINS.kD
            }
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
    }
