package frc.robot.subsystems.turret

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.Gains
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.intake.ligament
import frc.robot.subsystems.intake.mechanism
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LoggedOutput

@AutoLogOutput(key = "Turret/mechanism")
private var mechanism = LoggedMechanism2d(5.0, 5.0)
private var root = mechanism.getRoot("Turret", 2.5, 2.5)
private val ligament =
    root.append(LoggedMechanismLigament2d("TurretLigament", 1.0, 0.0))

object Turret : SubsystemBase() {
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            0,
            config = config,
            gearRatio = RATIO,
            simGains = Gains(kP = 1.0, kD = 0.0)
        )
    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    @LoggedOutput
    var setpoint: Angle = 0.degrees

    private fun setAngle(angle: Angle) = runOnce {
        setpoint = angle
        motor.setControl(positionVoltageRequest.withPosition(angle))
    }

    private fun addAngle(angle: Angle) = runOnce {
        setpoint += angle
        motor.setControl(positionVoltageRequest.withPosition(setpoint))
    }

    fun rotateClockwise() = addAngle(10.degrees) // Spinning Right

    fun rotateCounterClockwise() = addAngle((-10).degrees) // Spinning Left

    fun getAngle() = motor.inputs.position

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(setpoint[degrees])
        Logger.recordOutput("Turret", mechanism)
        Logger.processInputs("Subsystems/Turret", motor.inputs)
    }
}
