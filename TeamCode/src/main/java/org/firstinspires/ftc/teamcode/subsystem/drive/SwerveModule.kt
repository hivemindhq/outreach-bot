package org.firstinspires.ftc.teamcode.subsystem.drive

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.controller.PIDFController
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.CRServoImplEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.lib.hardware.AbsoluteAnalogEncoder
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.sign

data class SwerveModule(
    val motor: DcMotorEx,
    val servo: CRServo,
    val encoder: AbsoluteAnalogEncoder
) {
    var rotationController: PIDFController

    var position = 0.0
    var target = 0.0

    var wheelFlipped = false

    var lastError = 0.0
    var lastPower = 0.0
    var lastMotorPower = 0.0

    init {
        val configurationType = motor.motorType.clone()
        configurationType.achieveableMaxRPMFraction = maxMotor
        motor.motorType = configurationType
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        (servo as CRServoImplEx).pwmRange = PwmRange(500.0, 2500.00, 5000.0)

        rotationController = PIDFController(kP, kI, kD, 0.0)
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    fun read() {
        position = encoder.currentPosition
    }

    fun update() {
        rotationController.setPIDF(kP, kI, kD, 0.0)

        var target = getTargetPosition()
        val current = getModuleRotation()

        var error = normalizeRadians(target - current)

        if (motorFlipping && abs(error) > Math.PI / 2) {
            target = normalizeRadians(target - Math.PI)
            wheelFlipped = true
        } else {
            wheelFlipped = false
        }

        error = normalizeRadians(target - current)

        lastError = error

        var power = Range.clip(rotationController.calculate(0.0, error), -maxServo, maxServo)
        if (power.isNaN()) power = 0.0

        lastPower = power

        servo.power = (power + (if (abs(error) > 0.02) kStatic else 0.0) * power.sign)
    }

    fun getTargetPosition(): Double {
        return normalizeRadians(target - Math.PI)
    }

    fun getModuleRotation(): Double {
        return normalizeRadians(position - Math.PI)
    }

    fun setMotorPower(power: Double) {
        var power: Double = power
        if (wheelFlipped) power *= -1

        if (power != lastPower) {
            motor.power = power
        }

        lastMotorPower = power
    }

    fun setTargetRotatation(target: Double) {
        this.target = normalizeRadians(target)
    }

    fun getServoPower(): Double {
        return servo.power
    }

    fun getWheelVelocity(): Double {
        return ticksToIn(motor.velocity)
    }

    fun getLastServoError(): Double {
        return lastError
    }

    fun getLastPower(): Double {
        return lastPower
    }

    fun asState(): ModuleState {
        return ModuleState(this)
    }

    fun flipModifier(): Int {
        return if (wheelFlipped) -1 else 1
    }

    fun getWheelPosition(): Double {
        return ticksToIn(motor.currentPosition.toDouble())
    }

    fun setMode(runMode: DcMotor.RunMode) {
        motor.mode = runMode
    }

    fun serialize(name: String): String {
        return "$name | Motor Flipped: $wheelFlipped, Current Position: ${getModuleRotation()}, Target Position: ${getTargetPosition()}, Flip Modifier: ${flipModifier()}, Last Power: $lastMotorPower"
    }

    inner class ModuleState(val module: SwerveModule) {
        var wheelPosition = 0.0
        var podRotation = 0.0

        fun update(): ModuleState {
            return setState(-module.getWheelPosition(), -module.getTargetPosition())
        }

        fun setState(wheelPosition: Double, podRotation: Double): ModuleState {
            this.wheelPosition = wheelPosition
            this.podRotation = podRotation
            return this
        }

        fun calcDelta(): Vector2d {
            val oldWheel = wheelPosition
            update()
            return Vector2d.polar(wheelPosition - oldWheel, podRotation)
        }
    }

    fun ticksToIn(ticks: Double): Double {
        return wheelRadius * 2 * PI * gearRatio * ticks / ticksPerRev
    }

    @Config
    companion object SwerveConstants {
        @JvmField var wheelRadius = 1.25 // inches
        @JvmField var gearRatio = 1 / (1.83 * 1.5 * 2.88)
        @JvmField var ticksPerRev = 28

        @JvmField var motorFlipping: Boolean = true
        @JvmField var maxMotor = 1.0
        @JvmField var maxServo = 1.0

        @JvmField var kP = 0.6
        @JvmField var kI = 0.0
        @JvmField var kD = 0.12
        @JvmField var kStatic = 0.01
    }
}