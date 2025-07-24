package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units
import frc.robot.lib.Gains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.get

const val MAIN_MOTOR_PORT = 5
const val AUX_MOTOR_PORT = 6
val STATOR_CURRENT_LIMIT = 80.amps
val SUPPLY_CURRENT_LIMIT = 40.amps
val GAINS = Gains(kP = 1.0)
val STOP_VELOCITY = Units.RotationsPerSecond.zero()
val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }
        Slot0 = GAINS.toSlotConfig()

        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_CURRENT_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT[amps]
            }
    }
