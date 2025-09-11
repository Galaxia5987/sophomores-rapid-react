package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.robotstate.bindRobotCommands
import frc.robot.robotstate.robotDistanceFromHub
import frc.robot.robotstate.setIntakeing
import frc.robot.robotstate.turretAngleToHub
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.shooter.hood.HOOD_ANGLE_BY_DISTANCE
import frc.robot.subsystems.wrist.WristAngles
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {

    private val driverController = CommandPS5Controller(0)

    private val autoChooser: LoggedDashboardChooser<Command>

    var hoodAngle = InterpolatingDouble(robotDistanceFromHub[m])
    init {
        drive // Ensure Drive is initialized
        wrist.setAngle(WristAngles.DOWN.angle)
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                AutoBuilder.buildAutoChooser()
            )
        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
        bindRobotCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { driverController.leftX },
                { -driverController.leftY },
                { -driverController.leftY * 0.8 }
            )
        turret.defaultCommand = turret.setAngle { turretAngleToHub }
        hood.defaultCommand =
            hood.setAngle {
                hoodAngle.value = robotDistanceFromHub[m]
                HOOD_ANGLE_BY_DISTANCE.getInterpolated(hoodAngle).value.deg
            }
    }

    private fun configureButtonBindings() {

        // Switch to X pattern when X button is pressed

        driverController.circle().onTrue(setIntakeing())
        driverController.square().onTrue(wrist.setAngle(WristAngles.UP.angle))
        driverController.cross().onTrue(wrist.setAngle(WristAngles.DOWN.angle))
        // Reset gyro / odometry
        val resetOdometry =
            if (CURRENT_MODE == Mode.SIM)
                Runnable {
                    drive.resetOdometry(
                        driveSimulation!!.simulatedDriveTrainPose
                    )
                }
            else
                Runnable {
                    drive.resetOdometry(
                        Pose2d(drive.pose.translation, Rotation2d())
                    )
                }
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        val namedCommands: Map<String, Command> = mapOf()

        NamedCommands.registerCommands(namedCommands)

        // Set up SysId routines
        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization()
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )
    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

        drive.resetOdometry(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
