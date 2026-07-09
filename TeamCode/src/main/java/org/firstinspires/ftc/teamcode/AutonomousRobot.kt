package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.parts.IntakeMode

class AutonomousRobot(opMode: OpMode): Robot(opMode) {

    inner class SetIntakeIn(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            intake.set(IntakeMode.IN)
            return false
        }
    }

    inner class SetIntakeOut(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            intake.set(IntakeMode.OUT)
            return false
        }
    }

    inner class SetIntakeOff(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            intake.set(IntakeMode.OFF)
            return false
        }
    }

    inner class Update(): Action {
        override fun run(p: TelemetryPacket): Boolean {
            update()
            return true
        }
    }
}
