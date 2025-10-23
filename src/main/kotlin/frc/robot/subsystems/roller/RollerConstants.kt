package frc.robot.subsystems.roller

import edu.wpi.first.wpilibj.I2C
import frc.robot.lib.extensions.volts

const val MOTOR_PORT = 0
const val AUXILIARY_MOTOR_PORT = 1
const val SENSOR_ID = 11

val INTAKE = 12.volts
val OUTTAKE = -INTAKE
val STOP = 0.volts

val COLOR_SENSOR_PORT = I2C.Port.kOnboard
