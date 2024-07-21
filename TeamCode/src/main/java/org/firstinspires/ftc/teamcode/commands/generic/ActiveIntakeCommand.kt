package org.firstinspires.ftc.teamcode.commands.generic

import com.arcrobotics.ftclib.command.InstantCommand
import org.firstinspires.ftc.teamcode.OutreachBot

class ActiveIntakeCommand(power: Double): InstantCommand({
    OutreachBot.instance.intake.updateState(power)
})