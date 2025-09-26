package frc.robot.lib

import com.ctre.phoenix6.configs.Slot0Configs
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

data class Gains(
    val kP: Double = 0.0,
    val kI: Double = 0.0,
    val kD: Double = 0.0,
    val kS: Double = 0.0,
    val kV: Double = 0.0,
    val kA: Double = 0.0,
    val kG: Double = 0.0
) {
    /**
     * A function to convert a [Gains] type to the [Slot0Configs] that the motor
     * uses.
     */
    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@Gains.kP
            kI = this@Gains.kI
            kD = this@Gains.kD
            kA = this@Gains.kA
            kS = this@Gains.kS
            kV = this@Gains.kV
            kG = this@Gains.kG
        }
}

class LoggedNetworkGains(
    name: String,
    kP: Double = 0.0,
    kI: Double = 0.0,
    kD: Double = 0.0,
    kS: Double = 0.0,
    kV: Double = 0.0,
    kA: Double = 0.0,
    kG: Double = 0.0,
    key: String =
        (Throwable().stackTrace[1]?.fileName?.substringBeforeLast('.') + ""),
) {
    private val path = "/Tuning/$key/$name"
    val kP: LoggedNetworkNumber = LoggedNetworkNumber("$path/kP", kP)
    val kI: LoggedNetworkNumber = LoggedNetworkNumber("$path}/kI", kD)
    val kD: LoggedNetworkNumber = LoggedNetworkNumber("$path/kD", kI)
    val kS: LoggedNetworkNumber = LoggedNetworkNumber("$path/kS", kS)
    val kV: LoggedNetworkNumber = LoggedNetworkNumber("$path/kV", kV)
    val kA: LoggedNetworkNumber = LoggedNetworkNumber("$path/kA", kA)
    val kG: LoggedNetworkNumber = LoggedNetworkNumber("$path/kG", kG)

    fun toSlotConfig() =
        Slot0Configs().apply {
            kP = this@LoggedNetworkGains.kP.get()
            kI = this@LoggedNetworkGains.kI.get()
            kD = this@LoggedNetworkGains.kD.get()
            kA = this@LoggedNetworkGains.kA.get()
            kS = this@LoggedNetworkGains.kS.get()
            kV = this@LoggedNetworkGains.kV.get()
            kG = this@LoggedNetworkGains.kG.get()
        }
}
