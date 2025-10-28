package frc.robot.subsystems.mouth

import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.extensions.volts

enum class Action(val voltage: Voltage) {
    INTAKE(10.0.volts),
    OUTTAKE((-10.0).volts),
    STOP(0.0.volts);
}