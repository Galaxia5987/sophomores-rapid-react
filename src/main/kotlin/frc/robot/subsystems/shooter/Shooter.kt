package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.Gains
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LoggedOutput

@AutoLogOutput(key = "Shooter/mechanism")
private var mechanism = LoggedMechanism2d(5.0, 5.0)
private var root = mechanism.getRoot("Shooter", 2.5, 2.5)
private val ligament =
    root.append(LoggedMechanismLigament2d("ShooterLigament", 1.0, 0.0))

object Shooter : SubsystemBase() {
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            0,
            config = config,
            gearRatio = RATIO,
            simGains = Gains(kP = 1.0, kD = 0.0)
        )
    private val VoltageOutRequest: VelocityVoltage = VelocityVoltage(0.0)
    @LoggedOutput var setpoint: AngularVelocity = 0.rps

    private fun setVelocity(velocity: AngularVelocity) = runOnce {
        setpoint = velocity
        motor.setControl(VoltageOutRequest.withVelocity(velocity))
    }

    private fun setVelocity(velocitySupplier: () -> AngularVelocity) = runOnce {
        setpoint = velocitySupplier()
        motor.setControl(VoltageOutRequest.withVelocity(velocitySupplier()))
    }

    fun on(): Command = setVelocity(25.rps)

    fun off(): Command = setVelocity(0.rps)

    fun getMotorInputs() = motor.inputs

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(setpoint[rps])
        Logger.recordOutput("Shooter", mechanism)
        Logger.processInputs("Subsystems/Shooter", motor.inputs)
    }
}
