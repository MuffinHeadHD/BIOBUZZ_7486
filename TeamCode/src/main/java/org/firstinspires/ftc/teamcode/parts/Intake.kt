package org.firstinspires.ftc.teamcode.parts

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor

enum class IntakeModeFace { OFF_FACE, IN_FACE, OUT_FACE }
enum class IntakeModeTail { OFF_TAIL, IN_TAIL, OUT_TAIL}

class Intake(val intakeMotorFace: DcMotor, val intakeMotorTail: DcMotor, val colorSensFace: RevColorSensorV3) {
    fun set(mode: IntakeModeFace) {

        val intakePowerFace = when (mode) {
            IntakeModeFace.OFF_FACE ->  0.0
            IntakeModeFace.IN_FACE ->  -1.0
            IntakeModeFace.OUT_FACE ->  1.0
        }
        intakeMotorFace.power = intakePowerFace
    }
        fun set(mode: IntakeModeTail) {

            val intakePowerTail = when (mode) {
                IntakeModeTail.OFF_TAIL ->  0.0
                IntakeModeTail.IN_TAIL ->  -1.0
                IntakeModeTail.OUT_TAIL ->  1.0
            }
            intakeMotorTail.power = intakePowerTail
        }


}