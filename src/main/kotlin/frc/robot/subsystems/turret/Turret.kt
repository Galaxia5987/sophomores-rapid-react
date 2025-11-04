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
    private val motor: UniversalTalonFX = UniversalTalonFX(0, config = config, gearRatio = RATIO)
    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    private var targetPosition: Angle = 0.0.degrees

    private fun setAngle(angle: Angle): Command {
        return Commands.runOnce({
            targetPosition = angle
            motor.setControl(positionVoltageRequest.withPosition(angle))
        })
    }

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(motor.inputs.position[degrees])
        Logger.processInputs("Turret", motor.inputs)
        Logger.recordOutput("Turret/targetPosition", targetPosition)
        Logger.recordOutput("Subsystems/Turret/Ligament", mechanism)
    }
}
