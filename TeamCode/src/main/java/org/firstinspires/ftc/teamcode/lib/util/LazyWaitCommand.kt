package org.firstinspires.ftc.teamcode.lib.util

import com.arcrobotics.ftclib.command.CommandBase

class LazyWaitCommand(private val millis: Box<Double>): CommandBase() {
    private var startTime = 0.0

    override fun initialize() {
        startTime = Clock.milliseconds
    }

    override fun isFinished(): Boolean {
        return Clock.milliseconds - startTime > millis.get()
    }
}