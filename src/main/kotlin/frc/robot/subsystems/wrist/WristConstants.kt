package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

enum class Angles(degrees: Angle) {
    RESET(0.deg),
    CLOSE(0.deg),
    OPEN(0.deg)
}
