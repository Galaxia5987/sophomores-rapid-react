package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.auto.NamedCommands
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.autonomous.paths.deploy.pathplanner.AC1SRP
import frc.robot.autonomous.paths.deploy.pathplanner.BRP2
import frc.robot.autonomous.paths.deploy.pathplanner.CC2C3
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.shooting.toggleCompensation
import frc.robot.robotstate.bindRobotCommands
import frc.robot.robotstate.setForceShot
import frc.robot.robotstate.setIntaking
import frc.robot.robotstate.setOverrideDrive
import frc.robot.robotstate.setShooting
import frc.robot.robotstate.setStaticShooting
import frc.robot.robotstate.stopForceShot
import frc.robot.robotstate.stopOverrideDrive
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.roller.Roller
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {

    private val driverController = CommandPS5Controller(0)
    private val hidController = CommandGenericHID(1)
    private val autoChooser: LoggedDashboardChooser<Command>

    enum class HIDInput(val buttonId: Int) {
        DriveOverride(0),
        StaticSetpoint(1)
    }

    init {
        drive // Ensure Drive is initialized
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
                { -driverController.rightX * 0.8 }
            )
    }

    private fun configureButtonBindings() {

        // Switch to X pattern when X button is pressed
        driverController.circle().onTrue(setIntaking())
        driverController.L2().onTrue(Roller.intake()).onFalse(Roller.stop())
        driverController.R2().onTrue(Roller.outtake()).onFalse(Roller.stop())
        driverController.square().onTrue(setIntaking())
        driverController.cross().onTrue(setShooting())
        driverController.povUp().onTrue(toggleCompensation())
        driverController
            .triangle()
            .onTrue(setForceShot())
            .onFalse(stopForceShot())

        hidController
            .button(HIDInput.DriveOverride.buttonId)
            .whileTrue(setOverrideDrive())
            .whileFalse(stopOverrideDrive())
        hidController
            .button(HIDInput.StaticSetpoint.buttonId)
            .whileTrue(setStaticShooting())
            .whileFalse(setShooting())

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
        autoChooser.addDefaultOption("BRP2", BRP2())
        autoChooser.addOption("AC1SRP", AC1SRP())
        autoChooser.addOption("CC2C3", CC2C3())
    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

        drive.resetOdometry(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
