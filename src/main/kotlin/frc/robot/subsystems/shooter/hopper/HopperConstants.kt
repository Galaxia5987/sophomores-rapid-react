package frc.robot.subsystems.shooter.hopper

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.measure.Current
import edu.wpi.first.wpilibj.I2C
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.extensions.*

val MOTOR_ID = 4
val INTAKE_VOLTAGE = 4.volts

val STATOR_LIMIT = 30.amps
val SUPPLY_LIMIT: Current = STATOR_LIMIT * 2.0

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = STATOR_LIMIT[amps]
                SupplyCurrentLimit = SUPPLY_LIMIT[amps]
            }
    }

val DISTANCE_SENSOR_ID = 12

val DISTANCE_THRESHOLD = 50.mm

val RED_COLOR = Color(138, 87, 29)
val BLUE_COLOR = Color(73, 147, 33)
const val SIMILARITY_THRESHOLD = 0.9

val I2C_PORT = I2C.Port.kMXP
