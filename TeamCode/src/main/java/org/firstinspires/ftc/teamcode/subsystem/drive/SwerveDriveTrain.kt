package org.firstinspires.ftc.teamcode.subsystem.drive

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.util.plus
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.OutreachBot
import org.firstinspires.ftc.teamcode.lib.hardware.AbsoluteAnalogEncoder
import org.firstinspires.ftc.teamcode.lib.interfaces.DriveTrain
import org.firstinspires.ftc.teamcode.lib.interfaces.ISubsystem
import org.firstinspires.ftc.teamcode.lib.math.Pose
import org.firstinspires.ftc.teamcode.lib.types.Env.USE_WHEEL_FEEDFORWARD
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.sign

class SwerveDriveTrain(
    robot: OutreachBot
): DriveTrain, ISubsystem() {

    private val frontLeft: SwerveModule
    private val frontRight: SwerveModule
    private val backLeft: SwerveModule
    private val backRight: SwerveModule

    private val hardwareMap = robot.hardwareMap

    private var modules = arrayOf<SwerveModule>()

    private var R: Double
    var locked = false

    var wS: DoubleArray = DoubleArray(4)
    var wA: DoubleArray = DoubleArray(4)
    var max: Double = 0.0

    init {
        frontLeft = SwerveModule(
            motor("frontLeft"),
            rotation("frontLeftRotation"),
            encoder("frontLeftEncoder", frontLeftOffset)
        )
        frontRight = SwerveModule(
            motor("frontRight"),
            rotation("frontRightRotation"),
            encoder("frontRightEncoder", frontRightOffset)
        )
        backLeft = SwerveModule(
            motor("backLeft"),
            rotation("backLeftRotation"),
            encoder("backLeftEncoder", backLeftOffset)
        )
        backRight = SwerveModule(
            motor("backRight"),
            rotation("backRightRotation"),
            encoder("backRightEncoder", backRightOffset)
        )

        modules = arrayOf(frontLeft, frontRight, backLeft, backRight)

        modules.forEach { module ->
            module.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        }

        R = hypot(trackWidth, wheelBase)
    }

    override fun drive(pose: Pose) {
        val x = pose.x
        val y = pose.y
        val heading = pose.heading

        val a = x - heading * (wheelBase / R)
        val b = x + heading * (wheelBase / R)
        val c = y - heading * (trackWidth / R)
        val d = y + heading * (trackWidth / R)

        if (locked) {
            wS = doubleArrayOf(0.0, 0.0, 0.0, 0.0)
            wA = doubleArrayOf(Math.PI / 4, -Math.PI / 4, Math.PI / 4, -Math.PI / 4)
        } else {
            wS = doubleArrayOf(hypot(b, c), hypot(b, d), hypot(a, d), hypot(a, c))
            if (!maintainHeading) wA =
                doubleArrayOf(atan2(b, c), atan2(b, d), atan2(a, d), atan2(a, c))
        }

        max = max(wS)
    }

    override fun periodic() {}

    override fun read() {
        modules.forEach { module ->
            module.read()
        }
    }

    fun updateModules() {
        modules.forEach { module ->
            module.update()
        }
    }

    override fun write() {
        for (i in 0..3) {
            val m = modules[i]
            if (abs(max) > 1) wS[i] /= max
            m.setMotorPower(
                abs(wS[i]) + if (USE_WHEEL_FEEDFORWARD) {
                    minPow * wS[i].sign
                } else {
                    0.0
                }
            )
            m.setTargetRotatation(norm(wA[i]))
        }
    }

    override fun reset() {}

    private fun motor(name: String): DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name)

    private fun encoder(name: String, offset: Double): AbsoluteAnalogEncoder {
        val analog: AnalogInput = hardwareMap.get(AnalogInput::class.java, name)
        return AbsoluteAnalogEncoder(analog, 3.3).zero(offset).setInverted(true)
    }

    private fun rotation(name: String): CRServo = hardwareMap.get(CRServo::class.java, name)

    private fun max(args: DoubleArray): Double {
        var max = args[0]
        for (d in args) {
            if (d > max) max = d
        }
        return max
    }

    private fun norm(angle: Double): Double {
        return angle % (2 * Math.PI)
    }

    fun setLocked(locked: Boolean) {
        this.locked = locked
    }

    fun getTelemetry(): String {
        return frontLeft.serialize("frontLeft") + "\n" + frontRight.serialize("frontRight") + "\n" + backLeft.serialize("backLeft") + "\n" + backRight.serialize("backRight")
    }

    @Config
    companion object DriveConstants {
        @JvmField var trackWidth = 8.7
        @JvmField var wheelBase = 8.7

        @JvmField var maintainHeading = false

        const val minPow: Double = 0.1
        var imuOffset: Double = 0.0

        @JvmField var frontLeftOffset = 2.3
        @JvmField var frontRightOffset = 0.9
        @JvmField var backLeftOffset = 1.8
        @JvmField var backRightOffset = 3.6
    }
}