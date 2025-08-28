package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.deg

val SETPOINT_TOLERANCE = 1.0.deg
const val MOTOR_PORT = 2
const val GEAR_RATIO = 1 / 69.82

enum class WristAngles(val angle: Angle) {
    UP(70.deg),
    DOWN(30.deg)
}
