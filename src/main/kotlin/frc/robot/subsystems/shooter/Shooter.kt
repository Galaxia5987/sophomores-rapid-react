package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LoggedOutput

object Shooter: SubsystemBase() {
    @LoggedOutput private val motor: UniversalTalonFX = UniversalTalonFX(0, config = config, gearRatio = RATIO)
    @LoggedOutput private val VoltageOutRequest: VelocityVoltage = VelocityVoltage(0.0)
    @LoggedOutput private var setpoint: AngularVelocity = 0.rps

    private fun setVelocity(velocity: AngularVelocity) = runOnce {
        setpoint = velocity
        motor.setControl(VoltageOutRequest.withVelocity(velocity))
    }

    fun on() = runOnce { setVelocity(10.rps) }

    fun off() = runOnce { setVelocity(0.rps) }
}

