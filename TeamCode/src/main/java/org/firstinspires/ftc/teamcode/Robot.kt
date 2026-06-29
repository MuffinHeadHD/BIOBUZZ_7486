package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.parts.Drive
import org.firstinspires.ftc.teamcode.parts.Intake
import org.firstinspires.ftc.teamcode.util.GamepadState

@Config
class RobotConfig {

    companion object{

    }
}

open class Robot(val opMode: OpMode) {
    lateinit var left: DcMotor
    lateinit var right: DcMotor
    lateinit var driveMotors: Array<DcMotor>

    lateinit var intakeServoLeft: CRServo
    lateinit var intakeServoRight: CRServo
    lateinit var intakeMotor: DcMotor

    lateinit var drive: Drive
    lateinit var intake: Intake

    var dashboard: FtcDashboard = FtcDashboard.getInstance()
    var dashboardTelemetry: Telemetry = dashboard.telemetry

    lateinit var gamepadState1: GamepadState
    lateinit var lastGamepadState1: GamepadState
    lateinit var gamepadState2: GamepadState
    lateinit var lastGamepadState2: GamepadState

    init {
        val hwMap: HardwareMap = opMode.hardwareMap


        left = hwMap.get(DcMotor::class.java, "left")
        right = hwMap.get(DcMotor::class.java, "right")

        driveMotors = arrayOf(left, right)

        for (motor in driveMotors) {
            motor.power = 0.0
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

        left.direction = Direction.FORWARD
        right.direction = Direction.FORWARD

        intakeServoLeft = hwMap.get(CRServo::class.java, "intkL")
        intakeServoRight = hwMap.get(CRServo::class.java, "intkR")
        intakeMotor = hwMap.get(DcMotor::class.java, "intkMotor")


        drive = Drive(left, right)
        intake = Intake(intakeMotor, intakeServoLeft, intakeServoRight)


        gamepadState1 = GamepadState()
        lastGamepadState1 = GamepadState()
        gamepadState2 = GamepadState()
        lastGamepadState2 = GamepadState()
    }

    fun update() {
    }

    fun updateGamepadStates(last: Boolean) {
        if (last) {
            lastGamepadState1.updateGamepadState(gamepadState1)
            lastGamepadState2.updateGamepadState(gamepadState2)
        } else {
            gamepadState1.updateGamepadState(opMode.gamepad1)
            gamepadState2.updateGamepadState(opMode.gamepad2)
        }
    }
}