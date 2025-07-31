package frc.robot.robotstate

import edu.wpi.first.math.geometry.Ellipse2d
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.getPose2d


val HUB_LOCATION = getPose2d(8.2.m, 4.1.m)
const val COMMAND_NAME_PREFIX = "RobotCompositions"
val OUTER_DEAD_ZONE_AREA = Ellipse2d(HUB_LOCATION.translation, 4.m)
val INNER_DEAD_ZONE_AREA = Ellipse2d(HUB_LOCATION.translation, 0.2.m)