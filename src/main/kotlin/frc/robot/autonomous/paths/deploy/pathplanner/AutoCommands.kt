package frc.robot.autonomous.paths.deploy.pathplanner

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.CURRENT_MODE
import frc.robot.IS_RED
import frc.robot.Mode
import frc.robot.drive
import frc.robot.driveSimulation
import frc.robot.robotstate.RobotState

fun Spath(pathName: String, mirror: Boolean = false): Command =
    AutoBuilder.followPath(
        if (mirror) PathPlannerPath.fromPathFile(pathName).mirrorPath()
        else PathPlannerPath.fromPathFile(pathName)
    )
internal fun runPath(name: String): Command {
        val path = PathPlannerPath.fromPathFile(name)
        return Commands.runOnce({
            drive.resetOdometry(path.pathPoses[0])
            //if (CURRENT_MODE == Mode.SIM) {
             //   driveSimulation?.setSimulationWorldPose(path.pathPoses[0])
            //}
            AutoBuilder.resetOdom(path.pathPoses[0])
        }).andThen(
            AutoBuilder.followPath(path)
        )
    }


fun AC1(): Command = runPath("AC1")

fun C1S(): Command = runPath("C1S")

fun CC2(): Command = runPath("CC2")

fun C2C3(): Command = runPath("C2C3")

fun BRP2() : Command= runPath("BRP2")

fun AC1SRP(): Command =
    Commands.sequence(
        AC1(),
        C1S()
    )

fun CC2C3(): Command =
    Commands.sequence(
        CC2().until { RobotState.SHOOTING.equals(RobotState.SHOOTING) },
        C2C3().until { RobotState.SHOOTING.equals(RobotState.SHOOTING) }
    )
