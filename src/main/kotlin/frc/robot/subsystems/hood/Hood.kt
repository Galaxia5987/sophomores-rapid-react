package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionVoltage
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

object Hood : SubsystemBase() {

    @LoggedOutput(key = "Hood/mechanism")
    var mechanism = LoggedMechanism2d(4.0, 6.0)

    private var root = mechanism.getRoot("Hood", 2.0, 1.0)

    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))
    private val motor =
        UniversalTalonFX(
            port = port,
            config = config,
            gearRatio = GEAR_RATIO,
            simGains = simGains
        )
    private val positionReq: PositionVoltage = PositionVoltage(0.0)
    @LoggedOutput var setpoint: Angle = 0.deg

    fun setAngle(angle: Angle): Command {
        return Commands.runOnce({
            setpoint = angle
            motor.setControl(positionReq.withPosition(angle))
        })
    }

    fun moveUp(): Command {
        return setAngle(30.deg)
    }

    fun moveDown(): Command {
        return setAngle(0.deg)
    }

    fun getAngle() = motor.inputs.position

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(motor.inputs.position[degrees])
        Logger.processInputs("Hood", motor.inputs)
        Logger.recordOutput("Hood/targetAngle", Hood.setpoint)
        Logger.recordOutput("Hood/ligament", Hood.mechanism)
    }
}
