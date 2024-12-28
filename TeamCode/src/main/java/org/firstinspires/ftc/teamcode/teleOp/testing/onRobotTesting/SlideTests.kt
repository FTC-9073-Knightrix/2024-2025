package org.firstinspires.ftc.teamcode.teleOp.testing.onRobotTesting

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.teleOp.januaryComp.TeleOpHardwareMap
import kotlin.math.abs

@TeleOp(name = "VertSlideOuttakeTest")
class VertSlideOuttakeTest: TeleOpHardwareMap() {
    override fun loop() {
        TODO("Not yet implemented")
    }
}

@TeleOp(name = "HorizSlideIntakeTest")
class HorizSlideAndClawTest: TeleOpHardwareMap() {

    var intakeArmServoRot = 0.5
    override fun init() {
        super.init();
        // --------------------------------------- INITIALIZATION ---------------------------------------
        intakeClawServo.position = 1.0;

        horizLinearMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT // let slide be moved around
    }

    override fun loop() {
        telemetry.addData("Slide position", abs(horizLinearMotor.currentPosition))
        telemetry.addData("True pos:", horizLinearMotor.currentPosition)
        telemetry.addData("Power, direction", horizLinearMotor.power)
        telemetry.addData("Intake Arm position", intakeArmServoRot)
        telemetry.addData("Distance", intakeDistanceSensor.getDistance(DistanceUnit.CM)
        )
        horizLinearMotor.power = (gamepad1.right_stick_y * 0.25)
        if (gamepad1.dpad_up) {
            intakeArmServoRot += 0.001
        }
        if (gamepad1.dpad_down) {
            intakeArmServoRot -= 0.001
        }
        intakeArmServo.position = intakeArmServoRot
    }
}