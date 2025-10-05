package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DigitalInput
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rot
import frc.robot.lib.extensions.volts
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

@AutoLogOutput
private var mechanism = LoggedMechanism2d(6.0, 4.0)
private var root = mechanism.getRoot("Turret", 3.0, 2.0)
private val ligament =
    root.append(LoggedMechanismLigament2d("TurretLigament", 0.25, 90.0))

class Turret : SubsystemBase(), SysIdable {
    private val motor = UniversalTalonFX(MOTOR_ID, config = MOTOR_CONFIG)
    private val hallEffectSensor = DigitalInput(HALL_EFFECT_SENSOR_PORT)
    private val motionMagicTorque = MotionMagicTorqueCurrentFOC(0.0)
    private var angleSetpoint = 0.deg
    private val voltageRequest = VoltageOut(0.0)

    init {
        motor.reset(0.0.deg)
    }

    val isAtResetPoint =
        Trigger(hallEffectSensor::get).onTrue(runOnce { motor.reset(0.rot) })

    val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(angleSetpoint, TOLERANCE)
    }

    val disableTrigger =
        Trigger { DriverStation.isDisabled() }
            .onTrue(runOnce {
                MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Coast
                motor.applyConfiguration(
                    MOTOR_CONFIG
                )
            }
            )
//            .onFalse(setNeutralMode(NeutralModeValue.Brake))

    private fun setNeutralMode(neutralMode: NeutralModeValue): Command =
        runOnce {
            motor.applyConfiguration(
                MOTOR_CONFIG.apply {
                    MotorOutput.apply { NeutralMode = neutralMode }
                }
            )
        }

    fun setAngle(position: Angle) = runOnce {
        motor.setControl(motionMagicTorque.withPosition(position))
    }

    fun setAngle(position: () -> Angle) = run {
        angleSetpoint = position.invoke()
        motor.setControl(motionMagicTorque.withPosition(position.invoke()))
    }

    override fun periodic() {
        motor.updateInputs()
        ligament.setAngle(angleSetpoint.`in`(deg))
        Logger.processInputs("Subsystems/$name", motor.inputs)
        Logger.recordOutput("Subsystems/$name/isAtResetPoint", isAtResetPoint)
        Logger.recordOutput("Subsystems/$name/isAtSetpoint", isAtSetpoint)
        Logger.recordOutput("Subsystems/$name/Ligament", mechanism)
        Logger.recordOutput("Subsystems/$name/disableTrigger", disableTrigger)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage[volts]))
    }
}
