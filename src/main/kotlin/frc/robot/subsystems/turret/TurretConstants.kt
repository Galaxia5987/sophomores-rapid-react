package frc.robot.subsystems.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.extensions.get

const val RATIO = 56.0

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
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = 1.0 / 2.0 }
    }
