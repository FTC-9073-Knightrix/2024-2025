package org.firstinspires.ftc.teamcode.teleOp.testing.onRobotTesting

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name="Rumble Test")
class RumbleTest: OpMode() {
    override fun init() {
        telemetry.addData("Init", "Complete")
        telemetry.update()
    }

    override fun loop() {
        if (gamepad1.x) {
            gamepad1.rumble(10000)
            telemetry.addLine("g1 rumble" +
                    "")
        }
        if (gamepad2.x) {
            gamepad2.rumble(0.5, 0.5, 1000)
        }
        if (gamepad2.y) {
            gamepad1.rumble(0.5, 0.5, 1000)
            gamepad2.rumble(0.5, 0.5, 1000)

        }
    }
}