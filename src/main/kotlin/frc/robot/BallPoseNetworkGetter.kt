package frc.robot

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.networktables.StructArraySubscriber

const val tableKey = "RealsenseVision"
const val poses3dKey = "/$tableKey/poses"
val pose3dArray: StructArraySubscriber<Pose3d> by lazy {
    val inst = NetworkTableInstance.getDefault()
    val table = inst.getTable(tableKey)
    table.getStructArrayTopic(poses3dKey, Pose3d.struct).subscribe(arrayOf())
}
val robotRelativeBallPoses
    get() = pose3dArray.get()
