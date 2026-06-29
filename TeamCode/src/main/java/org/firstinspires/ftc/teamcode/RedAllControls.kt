package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.parts.IntakeMode


@TeleOp(name = "GoBilda starter bot", group = "BioBuzz")
class StarterBot : LinearOpMode() {
    lateinit var robot: Robot

    override fun runOpMode() {
        robot = Robot(this)

        waitForStart()

        while(opModeIsActive()) {

            robot.updateGamepadStates(false)
            robot.drive.drive(gamepad1.right_stick_x.toDouble(), -gamepad1.left_stick_y.toDouble())

            val intakeMode: IntakeMode = if (robot.gamepadState1.b) IntakeMode.IN else if (robot.gamepadState1.y) IntakeMode.OUT else IntakeMode.OFF
            robot.intake.set(intakeMode)


            robot.update()

            robot.updateGamepadStates(true)
        }
    }
}