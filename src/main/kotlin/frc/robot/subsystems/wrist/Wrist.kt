package frc.robot.subsystems.wrist

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LoggedOutput

object Wrist : SubsystemBase() {

    @LoggedOutput(key = "Wrist/mechanism")
    var mechanism = LoggedMechanism2d(4.0, 6.0)

    private var root = mechanism.getRoot("Wrist", 2.0, 1.0)

    private val ligament =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.0, 0.0))

    private val motor =
        UniversalTalonFX(
            port = port,
            simGains = simGains,
            config = config,
            gearRatio = GEAR_RATIO
        )
    private val positionReq1: PositionVoltage = PositionVoltage(0.0)
    @LoggedOutput var setpoint: Angle = Degrees.of(0.0)

    fun setAngle(angle: Angle): Command {
        return Commands.runOnce({
            setpoint = angle
            motor.setControl(positionReq1.withPosition(angle))
        })
    }

    fun open(): Command {
        return setAngle(25.deg)
    }

    fun close(): Command {
        return setAngle(0.deg)
    }

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(motor.inputs.position[degrees])
        Logger.processInputs("Wrist", motor.inputs)
        Logger.recordOutput("Wrist/targetAngle", setpoint)
        Logger.recordOutput("Wrist/ligament", mechanism)
    }
}
