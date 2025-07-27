package frc.robot.robotstate

import frc.robot.drive
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.m
import frc.robot.lib.getPose2d

val MinDistanceFromBasket = 0.5.m
val MaxDistanceFromBasket = 3.m
val basketLocation = getPose2d()
val RobotDistanceFromBasket
    get() = drive.pose.distanceFromPoint(basketLocation.translation)
