package frc.robot.subsystems.mouth

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

const val ratio = 5.0 / 8.0

val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimit = 75.0
                StatorCurrentLimitEnable = true
                SupplyCurrentLimit = 32.5
                SupplyCurrentLimitEnable = true
            }
    }


