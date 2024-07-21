package org.firstinspires.ftc.teamcode.commands.generic

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.lib.util.Box
import org.firstinspires.ftc.teamcode.lib.util.LazyWaitCommand
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem
import kotlin.math.abs

private const val MOVE_TIME = 300.0
private const val DEFAULT_MOVE_TIME = 500.0

fun clawCommand(state: DepositSubsystem.ClawState) = clawCommand(state, true)

fun clawCommand(state: DepositSubsystem.ClawState, withLock: Boolean): SequentialCommandGroup {
    val waitTime = Box(0.0)

    val res = SequentialCommandGroup(
        InstantCommand({
            val currState = OutreachBot.instance.deposit.clawState

            waitTime.set((if (currState != state) DEFAULT_MOVE_TIME else 0.0) + abs(state.pos - currState.pos) * MOVE_TIME)

            OutreachBot.instance.deposit.updateState(state)
            println("debug: (claw) wait time = ${waitTime.get()}ms")
        }),
        LazyWaitCommand(waitTime)
    )

    if (withLock) {
        res.addRequirements(OutreachBot.instance.deposit.armCommandSubsystem)
    }

    return res
}