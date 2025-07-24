package frc.robot.subsystems.wrist

import edu.wpi.first.units.measure.Angle
import frc.robot.lib.extensions.degrees

enum class Angles(degrees: Angle) {
    resetAngle(0.0.degrees),
    openAngle(0.0.degrees),
    closeAngle(0.0.degrees)
}
