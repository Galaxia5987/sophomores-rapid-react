package frc.robot.subsystems.shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Shooter: SubsystemBase() {
    private val motor: UniversalTalonFX = UniversalTalonFX(0, config = config, gearRatio = RATIO)
    private val VoltageOutRequest: VelocityVoltage = VelocityVoltage(0.0)
    private var setpoint: AngularVelocity = 0.rps

    private fun setVelocity(velocity: AngularVelocity): Command {
        return Commands.runOnce({
            setpoint = velocity
            motor.setControl(VoltageOutRequest.withVelocity(velocity))
        })
    }

    fun startShooting(): Command{
        return setVelocity(10.rps)
    }

    fun stopShooting(): Command{
        return setVelocity(0.rps)
    }

    override fun periodic(){
        motor.updateInputs()
        Logger.processInputs("Shooter", motor.inputs)
        Logger.recordOutput("Shooter/setpoint", setpoint)
    }
}

