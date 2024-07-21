package org.firstinspires.ftc.teamcode.commands.generic

import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitUntilCommand
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.subsystem.DepositSubsystem

fun slideCommand(state: DepositSubsystem.SlideState) = slideCommand(state, true)
fun slideCommand(state: DepositSubsystem.SlideState, withLock: Boolean): SequentialCommandGroup {
    var same = false

    val res = SequentialCommandGroup(
        InstantCommand({
            same = OutreachBot.instance.deposit.slideState == state

            if (same && state == DepositSubsystem.SlideState.DEPOSIT) {
                OutreachBot.instance.deposit.add()
            } else {
                OutreachBot.instance.deposit.updateState(state)
            }

        }),
        WaitUntilCommand { OutreachBot.instance.deposit.inThresh() }
    )

    if (withLock) {
        res.addRequirements(OutreachBot.instance.deposit.liftCommandSubsystem)
    }

    return res
}