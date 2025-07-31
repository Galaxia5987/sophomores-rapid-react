package frc.robot

import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.LEDPattern.gradient
import edu.wpi.first.wpilibj.LEDPattern.solid
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.lib.extensions.sec
import frc.robot.robotstate.hasFrontBall
import frc.robot.robotstate.isIdling
import frc.robot.robotstate.isInDeadZone
import frc.robot.robotstate.isIntaking
import frc.robot.robotstate.isShooting

const val STRIP_LENGTH = 41
const val LED_STRIP_PORT = 1

private val ledStrip =
    AddressableLED(LED_STRIP_PORT).apply {
        setLength(STRIP_LENGTH)
        start()
    }
private val ledBuffer = AddressableLEDBuffer(STRIP_LENGTH)

fun LEDPattern.applyLedPattern(): Command =
    Commands.runOnce({ this.applyTo(ledBuffer) })

fun Color.setLed() = solid(this).applyLedPattern()

fun applyLeds() {
    isShooting.onTrue(Color.kBlue.setLed())
    isIntaking.onTrue(Color.kPink.setLed())
    isIdling.onTrue(Color.kWhiteSmoke.setLed())
    isShooting
        .and(isInDeadZone.negate())
        .onTrue(solid(Color.kGreen).blink(1.sec).applyLedPattern())
    isIntaking
        .and(hasFrontBall)
        .onTrue(
            gradient(
                LEDPattern.GradientType.kDiscontinuous,
                Color.kPink,
                Color.kRed,
                Color.kOrange
            )
                .applyLedPattern()
        )
}
