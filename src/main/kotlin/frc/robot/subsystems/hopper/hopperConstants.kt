package frc.robot.subsystems.hopper

import com.ctre.phoenix.motorcontrol.NeutralMode
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains

val port = 5
val simGains = Gains(kP = 1.0, kD = 0.7)
val REAL_GAINS = Gains(kP = 1.0, 0.7)
val counfig =
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
                StatorCurrentLimit = 10.0
                SupplyCurrentLimit = 20.0
            }
        Slot0 =
            Slot0Configs().apply {
                kP = REAL_GAINS.kP
                kD = REAL_GAINS.kD
            }
    }
