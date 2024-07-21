package org.firstinspires.ftc.teamcode.lib.interfaces

import org.firstinspires.ftc.teamcode.lib.math.Pose

/**
 * A class representing an FTC drivetrain
 */
interface DriveTrain {

    /**
     * Drives the robot relative to the given pose.
     *
     * @param pose The pose to drive relative to
     */
    fun drive(pose: Pose)
}