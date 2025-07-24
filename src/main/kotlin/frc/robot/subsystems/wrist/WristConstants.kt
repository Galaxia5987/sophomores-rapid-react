package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.degrees

 const val GEAR_RATIO = 1/69.82

enum class Angles(degrees: Angle) {
    CLOSE(0.deg),
    OPEN(0.deg)
}
