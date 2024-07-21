package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.lib.interfaces.ISubsystem
import org.firstinspires.ftc.teamcode.lib.types.Env

class IntakeSubsystem(
    robot: OutreachBot
): ISubsystem() {

    private val hardwareMap = robot.hardwareMap

    var intakeMotor: DcMotorEx

    var lastPower = 0.0

    init {
        intakeMotor = motor("intake")
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        if (Env.USING_PHOTON) {
            intakeMotor.direction = DcMotorSimple.Direction.REVERSE
        }
    }

    fun updateState(power: Double) {
        if (lastPower != power) {
            intakeMotor.power = Range.clip(power, minSpeed, maxSpeed)
        }

        lastPower = power
    }

    override fun periodic() {}

    override fun read() {}

    override fun write() {}

    override fun reset() {}

    private fun motor(name: String): DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name)

    @Config
    companion object IntakeConfig {
        @JvmField var maxSpeed = 1.0
        @JvmField var minSpeed = -1.0
    }
}
