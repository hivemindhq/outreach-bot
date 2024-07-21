package org.firstinspires.ftc.teamcode.opmode

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians
import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.controller.PIDFController
import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.ktx.robot
import org.firstinspires.ftc.teamcode.lib.math.Point
import org.firstinspires.ftc.teamcode.lib.math.Pose
import org.firstinspires.ftc.teamcode.lib.math.SlewRateLimiter
import org.firstinspires.ftc.teamcode.lib.types.Alliance
import org.firstinspires.ftc.teamcode.lib.types.Env
import org.firstinspires.ftc.teamcode.lib.types.OpModeType
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.ln
import kotlin.math.pow
import kotlin.math.sign

@TeleOp(name = "teleop!", group = "!")
class Teleop: CommandOpMode() {
    lateinit var robot: OutreachBot

    lateinit var driver: GamepadEx
    lateinit var gunner: GamepadEx

    private var timer: ElapsedTime? = null

    private var loopTime = 0L

    val headingController: PIDFController = PIDFController(kP, kI, kD, 0.0)
    var lockHeading = false
    var targetHeading: Double = 0.0

    lateinit var forward: SlewRateLimiter
    lateinit var strafe: SlewRateLimiter

    override fun initialize() {
        CommandScheduler.getInstance().reset()

        robot = robot {
            startPose {
                x = 0.0
                y = 0.0
                heading = 0.0
            }
            opModeType(OpModeType.TELE_OP)
            hardwareMap(hardwareMap)
            alliance(Alliance.RED)
        }

        robot.init()

        driver = GamepadEx(gamepad1)
        gunner = GamepadEx(gamepad2)
    }

    override fun run() {
        super.run()

        if (timer == null) {
            timer = ElapsedTime()
            robot.startIMUThread(this)
            forward = SlewRateLimiter(forwardRange)
            strafe = SlewRateLimiter(strafeRange)
        }

        robot.read()

        if (gamepad1.right_stick_button && Env.USING_IMU) {
            robot.drivetrain.imuOffset = robot.imuAngle + PI
        }

        if (abs(gamepad1.right_stick_y) > 0.25) {
            lockHeading = true
            targetHeading = PI + robot.drivetrain.imuOffset
        }

        if (abs(gamepad2.right_stick_y) > -0.25) {
            lockHeading = true
            targetHeading = robot.drivetrain.imuOffset
        }

        val turn = joystickScalar(gamepad1.right_stick_x.toDouble(), 0.01)

        if (abs(turn) > 0.002) {
            lockHeading = false
        }

        val error = normalizeRadians(normalizeRadians(targetHeading) - normalizeRadians(robot.getAngle()))
        var headingControlSignal = -headingController.calculate(0.0, error * 12.4 / robot.voltage)

        if (abs(headingControlSignal) > 0.02) {
            headingControlSignal = 0.0
        }

        robot.drivetrain.maintainHeading = (
                abs(gamepad1.left_stick_x) < 0.002
                        && abs(gamepad1.left_stick_y) < 0.002
                        && abs(turn) < 0.002
                        && abs(headingControlSignal) < 0.002
                )

        val rotationAmount = if (Env.USING_IMU) robot.getAngle() - robot.drivetrain.imuOffset else 0

        var drive: Pose = Pose(
            Point(
                joystickScalar(gamepad1.left_stick_x.toDouble(), 0.001),
                joystickScalar(gamepad1.left_stick_y.toDouble(), 0.001)
            ).rotate(rotationAmount.toDouble()),
            if (lockHeading) headingControlSignal else turn
        )

        drive = Pose(
            forward.calculate(drive.y),
            strafe.calculate(drive.x),
            drive.heading
        )

        robot.intake.updateState(getIntakeSpeed())

        robot.loop(drive)
        robot.write()
        robot.clearBulkCache()

        val loop = System.currentTimeMillis()
        telemetry.addData("hz", 1000000000 / (loop - loopTime))
        loopTime = loop
        telemetry.update()
    }

    private fun joystickScalar(num: Double, min: Double): Double {
        return joystickScalar(num, min, 0.66, 4.0)
    }

    private fun joystickScalar(n: Double, m: Double, l: Double, a: Double): Double {
        return (sign(n) * m
                + (1 - m) *
                (if (abs(n) > l) abs(n).pow(ln(l / a) / ln(l)) * sign(n) else n / a))
    }

    private fun getIntakeSpeed(): Double {
        return driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
    }

    @Config
    companion object Teleop {
        @JvmField var kP = 0.0
        @JvmField var kI = 0.0
        @JvmField var kD = 0.0

        @JvmField var forwardRange = 4.0
        @JvmField var strafeRange = 4.0
    }
}