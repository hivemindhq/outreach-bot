package org.firstinspires.ftc.teamcode.subsystem

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.command.SubsystemBase
import com.arcrobotics.ftclib.controller.PIDController
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.arcrobotics.ftclib.hardware.motors.MotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.lib.interfaces.ISubsystem
import kotlin.math.abs

class DepositSubsystem(
    robot: OutreachBot
): ISubsystem() {
    val armCommandSubsystem = object : SubsystemBase() {}
    val liftCommandSubsystem = object : SubsystemBase() {}

    private val hardwareMap = robot.hardwareMap
    private var pidController: PIDController

    /**
     * Hardware
     */

    // Claw

    var claw: Servo
    var wrist: Servo

    // V4B

    var v4bPitch: Servo
    var v4bLeft: Servo
    var v4bRight: Servo

    // Lift

    var liftMotor: DcMotorEx
    var enc: Motor.Encoder

    /**
     * Telemetry & Constants
     */

    var pos: Int = 0
    var thresh = 20.0
    var target = 0
    var lastPIDOutput = 0.0
    var lastPower = 0.0

    var isAttemptingRetract = false
    var isAtBottom = false
    var isInDeposit = false

    var v4bState: V4BState = V4BState.TRANSFER
    var v4bPitchState: V4BState = V4BState.TRANSFER
    var slideState: SlideState = SlideState.BOTTOM
    var clawState: ClawState = ClawState.CLOSED

    var clawRotationIndex = 0


    init {
        pidController = PIDController(kP, kI, kD)

        claw = servo("claw")
        wrist = servo("wrist")
        v4bPitch = servo("v4bPitch")
        v4bLeft = servo("v4bLeft")
        v4bRight = servo("v4bRight")

        liftMotor = motor("lift")

        liftMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        enc = encoder("whatever")
    }

    fun inThresh(): Boolean {
        val res = abs(pos - target) < thresh
        println("deposit in threshold!")

        if (res) {
            isAtBottom = true
            isAttemptingRetract = false
        }

        return res
    }

    enum class V4BState(val pos: Double, val pitch: Double) {
        INSIDE_PIXEL(0.0, 0.0),
        TRANSFER(0.0, 0.0),
        DEPOSIT(0.0, 0.0)
    }

    enum class SlideState {
        BOTTOM, DEPOSIT
    }

    enum class ClawState(val pos: Double) {
        OPEN(0.0),
        PICKUP(0.0),
        CLOSED(0.0)
    }

    fun updateState(state: SlideState) {
        target = when (state) {
            SlideState.BOTTOM -> {
                bottom
            }
            SlideState.DEPOSIT -> {
                depositHeight
            }
        }
    }

    fun updateState(state: V4BState) {
        v4bState = state
        v4bLeft.position = state.pos
        v4bRight.position = state.pos
    }

    fun updatePitchState(state: V4BState) {
        v4bPitchState = state
        v4bPitch.position = state.pitch
    }

    fun updateState(state: ClawState) {
        clawState = state
        claw.position = state.pos
    }

    fun updateClawRotationIndex(increment: Boolean, turns: Int) {
        if (increment) {
            clawRotationIndex += turns
        } else {
            clawRotationIndex -= turns
        }

        wrist.position = ((clawRotationIndex * 30) / (355.0 / 2.0) + clawRotationOffset)
    }

    override fun periodic() {
        val power = if (inThresh() && !isAttemptingRetract) {
            0.0
        } else {
            val controlSignal = pidController.calculate(pos.toDouble(), target.toDouble())
            Range.clip(controlSignal, -1.0, 1.0)
        }

        lastPIDOutput = power
    }

    override fun read() {
        pos = enc.position
    }

    override fun write() {
        if (lastPIDOutput != lastPower) {
            liftMotor.power = lastPIDOutput
        }

        lastPower = lastPIDOutput
    }

    override fun reset() {}

    fun add() {
        target += ticksPerPixel
    }

    fun remove() {
        target -= ticksPerPixel
    }

    private fun servo(name: String): Servo = hardwareMap.get(Servo::class.java, name)
    private fun motor(name: String): DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name)
    private fun encoder(name: String): Motor.Encoder {
        return MotorEx(hardwareMap, name).encoder
    }

    @Config
    companion object LiftPIDConstants {
        @JvmField var kP = 0.0
        @JvmField var kI = 0.0
        @JvmField var kD = 0.0
        @JvmField var kF = 0.0
        @JvmField var ticksPerPixel = 0

        @JvmField var bottom = 0
        @JvmField var depositHeight = 0

        @JvmField var clawRotationOffset = 0.0
    }
}