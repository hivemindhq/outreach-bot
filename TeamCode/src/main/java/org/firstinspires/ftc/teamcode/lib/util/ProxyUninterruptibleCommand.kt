package org.firstinspires.ftc.teamcode.lib.util

import com.arcrobotics.ftclib.command.Command
import com.arcrobotics.ftclib.command.CommandBase

class ProxyUninterruptibleCommand(vararg val toSchedule: Command): CommandBase() {
    private var finished = false

    override fun initialize() {
        toSchedule.forEach { command ->
            command.schedule(false)
            if (!command.isScheduled) println("warning! uninterruptible command proxy failed to schedule $command")
        }
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) println("warning! uninterruptible command proxy was interrupted")
    }

    override fun execute() {
        finished = true
        toSchedule.forEach { command ->
            finished = finished && !command.isScheduled
        }
    }

    override fun isFinished(): Boolean {
        return finished
    }
}