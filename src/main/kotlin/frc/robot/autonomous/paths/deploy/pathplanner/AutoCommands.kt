package frc.robot.autonomous.paths.deploy.pathplanner

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.robotstate.RobotState

fun Spath(pathName: String, mirror: Boolean = false): Command =
    AutoBuilder.followPath(
        if (mirror) PathPlannerPath.fromPathFile(pathName).mirrorPath()
        else PathPlannerPath.fromPathFile(pathName)
    )

fun AC1(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("AC1"))
    )

fun C1S(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("C1S"))
    )

fun CC2(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("CC2"))
    )

fun C2C3(): Command =
    Commands.sequence(
        AutoBuilder.followPath(PathPlannerPath.fromPathFile("C2C3"))
    )

fun AC1SRP(): Command =
    Commands.sequence(
        AC1().until { RobotState.SHOOTING.equals(RobotState.SHOOTING) },
        C1S().until { RobotState.SHOOTING.equals(RobotState.SHOOTING) }
    )

fun CC2C3(): Command =
    Commands.sequence(
        CC2().until { RobotState.SHOOTING.equals(RobotState.SHOOTING) },
        C2C3().until { RobotState.SHOOTING.equals(RobotState.SHOOTING) }
    )
