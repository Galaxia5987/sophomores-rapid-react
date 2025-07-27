package frc.robot.robotstate

import frc.robot.drive
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.m
import frc.robot.lib.getPose2d

val minDistanceFromBasket = 0.5.m
val maxDistanceFromBasket = 3.m
val basketLocation = getPose2d()
val robotDistanceFromBasket
    get() = drive.pose.distanceFromPoint(basketLocation.translation)
