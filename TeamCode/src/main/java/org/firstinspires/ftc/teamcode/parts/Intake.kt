package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor

enum class IntakeMode { OFF, IN, OUT }

class Intake(val intakeMotor: DcMotor, val intakeServoLeft: CRServo, val intakeServoRight: CRServo) {
    fun set(mode: IntakeMode) {

        val intakePower: Double
        when(mode) {
            IntakeMode.OFF -> intakePower = 0.0
            IntakeMode.IN -> intakePower = -1.0
            IntakeMode.OUT -> intakePower = 1.0
        }
        intakeMotor.power = intakePower
        intakeServoLeft.power = intakePower
        intakeServoRight.power = intakePower
    }
}