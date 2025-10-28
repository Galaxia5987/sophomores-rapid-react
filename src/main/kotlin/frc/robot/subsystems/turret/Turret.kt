package frc.robot.subsystems.turret

import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX

object Turret: SubsystemBase() {
    private val motor1: UniversalTalonFX = UniversalTalonFX(0, config = config, gearRatio = ratio)
}