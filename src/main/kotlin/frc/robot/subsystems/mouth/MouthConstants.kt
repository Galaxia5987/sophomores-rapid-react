package frc.robot.subsystems.mouth

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue

const val ratio = 69.82

val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
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
