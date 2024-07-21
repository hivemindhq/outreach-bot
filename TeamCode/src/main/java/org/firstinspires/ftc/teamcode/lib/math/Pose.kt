package org.firstinspires.ftc.teamcode.lib.math

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import java.util.Locale

/**
 * A class representing a pose in a 2D space.
 *
 * @param x The x-coordinate of the pose.
 * @param y The y-coordinate of the pose.
 * @param heading The angle represented in the pose
 *
 * @author Mason Stuart (Power Play, 16379 KookyBotz)
 */
class Pose @JvmOverloads constructor(x: Double = 0.0, y: Double = 0.0, var heading: Double = 0.0) : Point(x, y) {
    constructor(p: Point, heading: Double) : this(p.x, p.y, heading)

    fun add(other: Pose): Pose {
        return Pose(x + other.x, y + other.y, heading + other.heading)
    }

    fun subtract(other: Pose): Pose {
        return Pose(
            this.x - other.x,
            this.y - other.y,
            AngleUnit.normalizeRadians(this.heading - other.heading)
        )
    }

    fun divide(other: Pose): Pose {
        return Pose(this.x / other.x, this.y / other.y, this.heading / other.heading)
    }

    fun subt(other: Pose): Pose {
        return Pose(x - other.x, y - other.y, heading - other.heading)
    }
}