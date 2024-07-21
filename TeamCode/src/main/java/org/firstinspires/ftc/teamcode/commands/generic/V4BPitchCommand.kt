package org.firstinspires.ftc.teamcode.commands.generic

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.lib.util.Box
import org.firstinspires.ftc.teamcode.lib.util.LazyWaitCommand
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem
import kotlin.math.abs

private const val MOVE_TIME = 1000L // ms
private const val MOVE_TIME_DEFAULT = 1000L

fun v4bPitchCommand(state: DepositSubsystem.V4BState) = v4bPitchCommand(state, true)
fun v4bPitchCommand(state: DepositSubsystem.V4BState, withLock: Boolean): SequentialCommandGroup {
    val waitTime = Box(0.0)

    val res = SequentialCommandGroup(
        InstantCommand({
            val currState = OutreachBot.instance.deposit.v4bState
            waitTime.set(
                (if (currState != state) MOVE_TIME_DEFAULT else 0L)
                        +  abs(state.pitch - (currState.pitch)) * MOVE_TIME
            )

            OutreachBot.instance.deposit.updatePitchState(state)
            println("debug: (v4b [pitch]) wait time = ${waitTime.get()}ms")
        }),
        LazyWaitCommand(waitTime)
    )

    if (withLock) {
        res.addRequirements(OutreachBot.instance.deposit.armCommandSubsystem)
    }

    return res
}