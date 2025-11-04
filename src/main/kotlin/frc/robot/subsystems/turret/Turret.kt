package frc.robot.subsystems.turret

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

@AutoLogOutput(key = "Turret/mechanism")
private var mechanism = LoggedMechanism2d(5.0, 5.0)
private var root = mechanism.getRoot("Turret", 2.5, 2.5)
private val ligament =
    root.append(LoggedMechanismLigament2d("TurretLigament", 1.0, 0.0))

object Turret : SubsystemBase() {
    private val motor1: UniversalTalonFX =
        UniversalTalonFX(0, config = config, gearRatio = ratio)
    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    private var setPoint: Angle = 0.0.degrees

    override fun periodic() {
        motor1.updateInputs()
        ligament.setAngle(motor1.inputs.position[degrees])
        Logger.processInputs("Turret", motor1.inputs)
        Logger.recordOutput("Turret/setPoint", setPoint)
        Logger.recordOutput("Subsystems/Turret/Ligament", mechanism)
    }

    fun setAngle(angle: Angle): Command {
        return Commands.runOnce({
            setPoint = angle
            motor1.setControl(positionVoltageRequest.withPosition(angle))
        })
    }
}
