package org.firstinspires.ftc.teamcode.lib.util


class Box<T>(private var t: T) {
    fun get(): T = t

    fun set(t: Double) {
        this.t = t
    }
}