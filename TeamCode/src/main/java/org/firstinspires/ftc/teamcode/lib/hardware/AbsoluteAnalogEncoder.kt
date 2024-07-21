package org.firstinspires.ftc.teamcode.lib.hardware

import com.qualcomm.robotcore.hardware.AnalogInput
import kotlin.math.abs

class AbsoluteAnalogEncoder @JvmOverloads constructor(val encoder: AnalogInput, private val analogRange: Double = defaultRange) {
    private var offset = 0.0

    var direction = false
        private set

    private var wrapAround = false

    fun zero(offset: Double): AbsoluteAnalogEncoder {
        this.offset = offset
        return this
    }

    fun setInverted(inverted: Boolean): AbsoluteAnalogEncoder {
        direction = inverted
        return this
    }

    fun setWraparound(wrapAround: Boolean): AbsoluteAnalogEncoder {
        this.wrapAround = wrapAround
        return this
    }

    private var pastPosition = 1.0


    /**
     * Returns [angle] clamped to `[0, 2pi]`.
     *
     * @param angle angle measure in radians
     */
    fun norm(angle: Double): Double {
        val TAU = Math.PI * 2
        var modifiedAngle = angle % TAU

        modifiedAngle = (modifiedAngle + TAU) % TAU

        return modifiedAngle
    }

    /**
     * Returns [angleDelta] clamped to `[-pi, pi]`.
     *
     * @param angleDelta angle delta in radians
     */
    fun normDelta(angleDelta: Double): Double {
        var modifiedAngleDelta = norm(angleDelta)

        if (modifiedAngleDelta > Math.PI) {
            modifiedAngleDelta -= Math.PI * 2
        }

        return modifiedAngleDelta
    }

    val currentPosition: Double
        get() {
            var position: Double = norm((if (!direction) 1 - voltage / analogRange else voltage / analogRange) * Math.PI * 2 - offset)
            if (wrapAround && position > Math.PI * 1.5) {
                position -= Math.PI * 2
            }
            if (!valueRejection || abs(normDelta(pastPosition)) > 0.1 || abs(normDelta(position)) < 1) {
                pastPosition = position
            }
            return pastPosition
        }

    val voltage: Double
        get() = encoder.voltage

    fun disable() {}

    val deviceType: String?
        get() = null

    companion object {
        var defaultRange = 3.3
        var valueRejection = false
    }
}