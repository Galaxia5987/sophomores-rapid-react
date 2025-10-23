package frc.robot.lib.extensions

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BooleanSupplier

operator fun Trigger.not() = this.negate()

operator fun Trigger.get(seconds: Time) =
    this.debounce(seconds.`in`(Units.Seconds))

fun Trigger.debounce(seconds: Time) = this[seconds]

fun Trigger.and(vararg trigger: BooleanSupplier): Trigger =
    trigger.fold(this) { baseTrigger, trigger -> baseTrigger.and(trigger) }

fun Trigger.or(vararg trigger: BooleanSupplier): Trigger =
    trigger.fold(this) { baseTrigger, trigger -> baseTrigger.or(trigger) }

fun Trigger.onTrue(vararg commands: Command): Trigger =
    commands.fold(this) { baseTrigger, command -> baseTrigger.onTrue(command) }

fun Trigger.onTrueUntil(trigger: Trigger, vararg commands: Command) {
    commands.forEach { this.onTrue(it.until(trigger)) }
}

fun Trigger.onFalse(vararg commands: Command): Trigger =
    commands.fold(this) { baseTrigger, command -> baseTrigger.onFalse(command) }

fun Trigger.whileTrue(vararg commands: Command): Trigger =
    commands.fold(this) { baseTrigger, command ->
        baseTrigger.whileTrue(command)
    }

fun Trigger.whileFalse(vararg commands: Command): Trigger =
    commands.fold(this) { baseTrigger, command ->
        baseTrigger.whileFalse(command)
    }
