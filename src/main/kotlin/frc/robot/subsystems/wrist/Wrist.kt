package frc.robot.subsystems.wrist

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.kilogramSquareMeters
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

private const val GEAR_RATIO = 1/69.82
val POINTE_TOLERANCE = 1.0.degrees
val MOTOR_PORT = 2
@AutoLogOutput private var mechanism = LoggedMechanism2d(6.0, 4.0)
private var root = mechanism.getRoot("Wrist", 3.0, 2.0)
private val ligament =
    root.append(LoggedMechanismLigament2d("WristLigament", 0.25, 90.0))

class Wrist : SubsystemBase() {
    @AutoLogOutput private var setpoint: Angle = 0.degrees

    private val motor =
        UniversalTalonFX(
            MOTOR_PORT,
            momentOfInertia = (0.0025).kilogramSquareMeters,
            gearRatio = GEAR_RATIO,
            config = TalonFXConfiguration()
        )

    private val positionRequest = PositionVoltage(0.0)

    fun setAngle(angle: Angle): Command = runOnce {
        setpoint = angle
        motor.setControl(positionRequest.withPosition(angle))
    }

    fun reset(): Command = runOnce { setAngle(0.0.degrees) }

    var atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, POINTE_TOLERANCE)
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
