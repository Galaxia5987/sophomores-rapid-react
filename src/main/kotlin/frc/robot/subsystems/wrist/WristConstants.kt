package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get


val SETPOINT_TOLERANCE = 1.0.deg
const val MOTOR_PORT = 2
const val GEAR_RATIO = 1 / 69.82

enum class WristAngles(val angle: Angle) {
    UP(70.deg),
    DOWN(30.deg)
}

val STATOR_CURRENT_LIMIT = 100.amps
val SUPPLY_CURRENT_LIMIT = 50.amps
val GAINS = Gains(kP = 2.0)
val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
            }
        Feedback = FeedbackConfigs().withRotorToSensorRatio(1.0)
        Slot0 = GAINS.toSlotConfig()

        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_CURRENT_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT[amps]
            }
    }