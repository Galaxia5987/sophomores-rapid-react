package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

const val STRIP_LENGTH = 41
const val LED_STRIP_PORT = 1

enum class StateColor(val pattern: LEDPattern) {
    IDLING(LEDPattern.solid(Color.kWhiteSmoke)),
    INTAKING(LEDPattern.solid(Color.kPink)),
    SHOOTING(LEDPattern.solid(Color.kBlue)),
}
