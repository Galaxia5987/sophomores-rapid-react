package frc.robot.robotstate

import org.team5987.annotation.LoggedOutput

enum class RobotState() {
    IDLING,
    INTAKING,
    SHOOTING
}

@LoggedOutput var state = RobotState.IDLING
