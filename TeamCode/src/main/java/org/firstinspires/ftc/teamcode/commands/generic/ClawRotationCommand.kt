package org.firstinspires.ftc.teamcode.commands.generic

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.lib.util.Box
import org.firstinspires.ftc.teamcode.lib.util.LazyWaitCommand
import kotlin.math.abs

private const val MOVE_TIME = 300 // ms

fun clawRotationCommand(increment: Boolean, turns: Int) = clawRotationCommand(increment, turns, true)
fun clawRotationCommand(increment: Boolean, turns: Int, withLock: Boolean): SequentialCommandGroup {
    val waitTime = Box(0.0)

    val res = SequentialCommandGroup(
        InstantCommand({
            waitTime.set(
                (abs(if (increment) turns else turns - OutreachBot.instance.deposit.clawRotationIndex) * MOVE_TIME).toDouble()
            )
            OutreachBot.instance.deposit.updateClawRotationIndex(increment, turns)
        }),
        LazyWaitCommand(waitTime)
    )

    if (withLock) {
        res.addRequirements(OutreachBot.instance.deposit.armCommandSubsystem)
    }

    return res
}