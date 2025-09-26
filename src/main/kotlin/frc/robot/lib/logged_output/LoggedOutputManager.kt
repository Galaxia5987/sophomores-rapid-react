package frc.robot.lib.logged_output

import edu.wpi.first.units.Measure
import edu.wpi.first.util.WPISerializable
import edu.wpi.first.util.struct.StructSerializable
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.toPrimitiveTypeJava
import frc.robot.lib.ifNotNull
import java.util.function.*
import kotlin.reflect.KFunction
import kotlin.reflect.KProperty0
import kotlin.reflect.jvm.javaGetter
import kotlin.reflect.jvm.javaMethod
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d

object LoggedOutputManager : SubsystemBase() {
    private val callbacks = mutableListOf<Runnable>()

    override fun periodic() = callbacks.forEach { it.run() }

    private fun makeKey(
        key: String,
        path: String = "",
        name: String,
        declaringClass: String?
    ): String {
        return if (path.isBlank())
            key.ifBlank { "${ declaringClass ?: "<unknown>" }/$name" }
        else path + key.ifBlank { "/$name" }
    }

    fun <T> registerField(
        key: String,
        property: KProperty0<T>,
        path: String = ""
    ) {
        val declaringClass = property.javaGetter?.declaringClass?.simpleName
        val actualKey = makeKey(key, path, property.name, declaringClass)
        register(actualKey, property::get)
    }

    fun <T> registerMethod(
        key: String,
        function: KFunction<T>,
        path: String = "",
    ) {
        if (function.parameters.isNotEmpty()) {
            throw IllegalArgumentException(
                "Only zero-arg functions are supported: $key"
            )
        }

        val declaringClass =
            function.javaMethod?.declaringClass?.simpleName ?: "<top-level>"
        val actualKey = makeKey(key, path, function.name, declaringClass)
        register(actualKey, function::call)
    }

    // Taken from advantageKit's `AutoLogOutputManager`,
    // https://github.com/rakrakon/AdvantageKit/blob/main/akit/src/main/java/org/littletonrobotics/junction/AutoLogOutputManager.java
    fun addRunnable(action: () -> Unit) {
        callbacks.add(Runnable(action))
    }

    @Suppress("UNCHECKED_CAST")
    private fun register(key: String, supplier: Supplier<*>) {
        fun value()= supplier.get()
        val type = value()::class.java.toPrimitiveTypeJava()!!
        if (!type.isArray) {
            // Single types
            when {
                type == Boolean::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Boolean)
                        }
                    }
                type == Int::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull { Logger.recordOutput(key, it as Int) }
                    }
                type == Long::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull { Logger.recordOutput(key, it as Long) }
                    }
                type == Float::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Float)
                        }
                    }
                type == Double::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Double)
                        }
                    }
                type == String::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as String?)
                        }
                    }
                type == LoggedMechanism2d::class.java ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(
                                key,
                                value() as LoggedMechanism2d?
                            )
                        }
                    }
                type.isEnum ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, (it as Enum<*>).name)
                        }
                    }
                type.isRecord ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Record)
                        }
                    }
                BooleanSupplier::class.java.isAssignableFrom(type) ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as BooleanSupplier?)
                        }
                    }
                IntSupplier::class.java.isAssignableFrom(type) ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as IntSupplier?)
                        }
                    }
                LongSupplier::class.java.isAssignableFrom(type) ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as LongSupplier?)
                        }
                    }
                DoubleSupplier::class.java.isAssignableFrom(type) ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as DoubleSupplier?)
                        }
                    }
                Measure::class.java.isAssignableFrom(type) ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Measure<*>?)
                        }
                    }
                else -> {
                    addRunnable {
                        value().ifNotNull {
                            try {
                                Logger.recordOutput(
                                    key,
                                    value() as WPISerializable
                                )
                            } catch (e: ClassCastException) {
                                DriverStation.reportError(
                                    "[LoggedOutputManager] Auto serialization is not supported for type " +
                                            type.getSimpleName(),
                                    false
                                )
                            }
                        }
                    }
                }
            }
        } else if (!type.componentType.isArray) {
            // Array types
            val componentType = type.componentType
            when {
                componentType == Byte::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as ByteArray?)
                        }
                    }
                componentType == Boolean::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Boolean)
                        }
                    }
                componentType == Int::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull { Logger.recordOutput(key, it as Int) }
                    }
                componentType == Long::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull { Logger.recordOutput(key, it as Long) }
                    }
                componentType == Float::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Float)
                        }
                    }
                componentType == Double::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Double)
                        }
                    }
                componentType == String::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as String?)
                        }
                    }
                componentType.isEnum ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, (it as Enum<*>).name)
                        }
                    }
                componentType.isRecord ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Record)
                        }
                    }
                else -> {
                    addRunnable {
                        value().ifNotNull {
                            try {
                                Logger.recordOutput(
                                    key,
                                    *value() as Array<StructSerializable?>
                                )
                            } catch (e: ClassCastException) {
                                DriverStation.reportError(
                                    "[LoggedOutputManager] Auto serialization is not supported for array type " +
                                            componentType.getSimpleName(),
                                    false
                                )
                            }
                        }
                    }
                }
            }
        } else {
            // 2D array types
            val componentType = type.componentType.componentType
            when {
                componentType == Byte::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as ByteArray?)
                        }
                    }
                componentType == Boolean::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Boolean)
                        }
                    }
                componentType == Int::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull { Logger.recordOutput(key, it as Int) }
                    }
                componentType == Long::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull { Logger.recordOutput(key, it as Long) }
                    }
                componentType == Float::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Float)
                        }
                    }
                componentType == Double::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Double)
                        }
                    }
                componentType == String::class.javaPrimitiveType ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as String?)
                        }
                    }
                componentType.isEnum ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, (it as Enum<*>).name)
                        }
                    }
                componentType.isRecord ->
                    addRunnable {
                        value().ifNotNull {
                            Logger.recordOutput(key, it as Record)
                        }
                    }
                else -> {
                    addRunnable {
                        value().ifNotNull {
                            try {
                                Logger.recordOutput(
                                    key,
                                    it as Array<Array<StructSerializable>?>?
                                )
                            } catch (e: ClassCastException) {
                                DriverStation.reportError(
                                    ("[LoggedOutputManager] Auto serialization is not supported for 2D array type " +
                                            componentType.getSimpleName()),
                                    false
                                )
                            }
                        }
                    }
                }
            }
        }
    }
}
