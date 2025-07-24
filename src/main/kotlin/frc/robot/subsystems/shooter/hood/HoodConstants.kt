package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Current
import frc.robot.lib.Gains
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get

// val MOTOR_ID: Nothing = TODO("Define motor id")

val MOTOR_ID = 0

val SETPOINT_TOLERANCE = 0.5.deg

val STATOR_LIMIT = 30.amps
val SUPPLY_LIMIT: Current = STATOR_LIMIT * 2.0
val PID_GAINS = Gains(kP = 1.0)

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
    }

// val ENCODER_ID: Nothing = TODO("Define encoder id")

val ENCODER_ID = 1

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
        MagnetSensor.MagnetOffset = 0.0
    }
