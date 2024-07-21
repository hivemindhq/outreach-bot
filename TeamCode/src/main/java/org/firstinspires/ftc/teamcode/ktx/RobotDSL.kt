package org.firstinspires.ftc.teamcode.ktx

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.lib.math.Pose
import org.firstinspires.ftc.teamcode.lib.types.Alliance
import org.firstinspires.ftc.teamcode.lib.types.OpModeType

class RobotBuilder {
    private lateinit var startPose: Pose
    private lateinit var opModeType: OpModeType
    private lateinit var hardwareMap: HardwareMap
    private lateinit var alliance: Alliance
    private lateinit var telemetry: Telemetry

    fun startPose(init: PoseBuilder.() -> Unit) {
        startPose = PoseBuilder().apply(init).build()
    }

    fun opModeType(type: OpModeType) {
        opModeType = type
    }

    fun hardwareMap(hardwareMap: HardwareMap) {
        this.hardwareMap = hardwareMap
    }

    fun alliance(alliance: Alliance) {
        this.alliance = alliance
    }

    fun telemetry(telemetry: Telemetry) {
        this.telemetry = telemetry
    }

    fun build(): OutreachBot {
        return OutreachBot(startPose, opModeType, hardwareMap, alliance, telemetry)
    }
}

class PoseBuilder() {
    var x: Double = 0.0
    var y: Double = 0.0
    var heading: Double = 0.0

    fun build(): Pose {
        return Pose(x, y, heading)
    }
}

fun robot(init: RobotBuilder.() -> Unit): OutreachBot {
    return RobotBuilder().apply(init).build()
}