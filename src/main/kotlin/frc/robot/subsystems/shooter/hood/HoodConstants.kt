package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Current
import frc.robot.lib.Gains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get

const val MOTOR_ID = 3

val SETPOINT_TOLERANCE = 0.5.deg

val STATOR_LIMIT = 30.amps
val SUPPLY_LIMIT: Current = STATOR_LIMIT * 2.0
val PID_GAINS = Gains(kP = 1.0)

const val ENCODER_ID = 10
const val ENCODER_TO_MECHANISM_RATIO = 1.0
const val MOTOR_TO_SENSOR_RATIO = 1.0

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = PID_GAINS.toSlotConfig()
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_LIMIT[amps]
            }

        Feedback =
            FeedbackConfigs().apply {
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
                FeedbackRemoteSensorID = ENCODER_ID
                SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO
                RotorToSensorRatio = MOTOR_TO_SENSOR_RATIO
            }
    }

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
        MagnetSensor.MagnetOffset = 0.0
    }
