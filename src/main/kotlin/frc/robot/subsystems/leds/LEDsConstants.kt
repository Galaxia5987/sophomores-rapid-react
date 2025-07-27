package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.LEDPattern
import edu.wpi.first.wpilibj.util.Color

const val STRIP_LENGTH = 41
const val LED_STRIP_PORT = 1

enum class StateColors(val pattern: LEDPattern) {
    Idle(LEDPattern.solid(Color.kWhiteSmoke)),
    Intaking(LEDPattern.solid(Color.kPink)),
    Shooting(LEDPattern.solid(Color.kBlue)),
}
