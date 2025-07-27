package frc.robot.subsystems.leds

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands

private val ledStrip =
    AddressableLED(LED_STRIP_PORT).apply {
        setLength(STRIP_LENGTH)
        start()
    }
private val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)

fun applyPattern(pattern: LEDPattern): Command = Commands.runOnce({
    pattern.applyTo(ledBuffer)
})

fun STATE_COLOR.applyPattern() {
    this.pattern.applyTo(ledBuffer)
}