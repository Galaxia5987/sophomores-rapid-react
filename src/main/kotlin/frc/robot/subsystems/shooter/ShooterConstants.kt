package frc.robot.subsystems.shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

const val ratio = 0.0

val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimit = 0.0
                StatorCurrentLimitEnable = true
                SupplyCurrentLimit = 0.0
                SupplyCurrentLimitEnable = true
            }
        Slot0 =
            Slot0Configs().apply {
                kP = 0.0
                kD = 0.0
            }
    }