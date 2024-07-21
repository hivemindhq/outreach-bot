package org.firstinspires.ftc.teamcode.lib.math

import androidx.core.math.MathUtils
import com.qualcomm.robotcore.util.ElapsedTime


class SlewRateLimiter(
    private val positiveRateLimit: Double,
    private val negativeRateLimit: Double,
    private var previousValue: Double
) {
    private val timer = ElapsedTime()
    private var previousTime = 0.0

    constructor(rateLimit: Double, initialValue: Double) : this(rateLimit, -rateLimit, initialValue)

    constructor(rateLimit: Double) : this(rateLimit, -rateLimit, 0.0)

    fun calculate(input: Double): Double {
        val currentTime = timer.seconds()
        val elapsedTime = currentTime - previousTime
        previousValue += MathUtils.clamp(
            input - previousValue,
            negativeRateLimit * elapsedTime,
            positiveRateLimit * elapsedTime
        )
        previousTime = currentTime
        return previousValue
    }
}