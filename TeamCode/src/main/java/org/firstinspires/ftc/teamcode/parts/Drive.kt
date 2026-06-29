package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.abs

class Drive(val left: DcMotor, val right: DcMotor) {
    var lastTime = System.nanoTime()

    fun drive(x: Double, y: Double) {
        val now = System.nanoTime()
        val dt = (now - lastTime) / 1e9
        lastTime = now

        val denominator = maxOf(abs(y) + abs(x), 1.0)

        val leftTarget = (y + x) / denominator
        val rightTarget = (y - x) / denominator

        val drivespeed = 1.0

        val rightOut = rightTarget * drivespeed
        val leftOut = leftTarget * drivespeed

        right.power = rightOut
        left.power = leftOut

    }
}