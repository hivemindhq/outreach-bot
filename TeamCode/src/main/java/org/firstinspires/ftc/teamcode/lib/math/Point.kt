package org.firstinspires.ftc.teamcode.lib.math

import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * A class representing a point in a 2D space.
 *
 * @param x The x-coordinate of the point.
 * @param y The y-coordinate of the point.
 *
 * @author Mason Stuart (Power Play, 16379 KookyBotz)
 */
open class Point @JvmOverloads constructor(var x: Double = 0.0, var y: Double = 0.0) {
    fun subtract(other: Point): Point {
        return Point(x - other.x, y - other.y)
    }

    fun add(other: Point): Point {
        return Point(x + other.x, y + other.y)
    }

    fun add(scalar: Double): Point {
        return Point(x + scalar, y + scalar)
    }

    fun subt(other: Point): Point {
        return Point(x - other.x, y - other.y)
    }

    fun divide(div: Double): Point {
        return Point(x / div, y / div)
    }

    fun distanceTo(other: Point): Double {
        return other.subtract(this).radius()
    }

    fun atan(): Double {
        return atan2(x, y)
    }

    fun radius(): Double {
        return hypot(x, y)
    }

    fun rotate(amount: Double): Point {
        return polar(radius(), atan() + amount)
    }

    companion object {
        fun polar(r: Double, a: Double): Point {
            return Point(cos(a) * r, sin(a) * r)
        }
    }
}