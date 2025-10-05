package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get

val SETPOINT_TOLERANCE = 1.0.deg
const val MOTOR_PORT = 2
const val GEAR_RATIO = 1 / 69.82

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = Slot0Configs().apply { kP = 0.8 }
        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 20.0
                SupplyCurrentLimit = 40.0
            }

        SoftwareLimitSwitch =
            SoftwareLimitSwitchConfigs().apply {
                ForwardSoftLimitEnable = true
                ForwardSoftLimitThreshold = 15.5
                ReverseSoftLimitEnable = true
            }
    }

enum class Angles(degrees: Angle) {
    CLOSE(0.deg),
    OPEN(0.deg)
}
