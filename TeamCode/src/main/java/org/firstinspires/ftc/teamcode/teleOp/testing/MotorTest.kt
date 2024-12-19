package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor

@TeleOp(name = "Motor + Encoder Test")
class MotorTest : OpMode() {
    lateinit var motor: DcMotor
    var motorPower: Double = 0.0;
    var encoderTarget: Int = 0;
    enum class MotorState {
        MANUAL,
        ENCODER
    }
    var motorState: MotorState = MotorState.MANUAL
    var fromStateChange: Boolean = false;

    override fun init() {
        motor = hardwareMap.get(DcMotor::class.java, "motor")
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    override fun loop() {
        // TEST MOTOR POWER ONLY
        if (motorState == MotorState.MANUAL) {
            if (fromStateChange) { motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER}

            motorPower = (gamepad1.left_trigger - gamepad1.right_trigger).toDouble()
            motor.power = motorPower

            // Switch to encoder testing
            if (gamepad1.a) {
                motorState = MotorState.ENCODER
                fromStateChange = true
            }
        }

        // TEST RUN TO POSIITION
        else if (motorState == MotorState.ENCODER) {
            if (fromStateChange) {motor.mode = DcMotor.RunMode.RUN_TO_POSITION }

            // Adjust encoder targets
            if (gamepad1.right_bumper) {encoderTarget++}
            if (gamepad1.left_bumper) {encoderTarget--}
            if (gamepad1.b) {encoderTarget = 0}

            // Adjust motor power variable
            if (gamepad1.dpad_left) {motorPower -= 0.01}
            if (gamepad1.dpad_right) {motorPower += 0.01}

            if (gamepad1.x) {
                motor.targetPosition = encoderTarget
                motor.power = motorPower
            }

            // Switch to manual testing
            if (gamepad1.y) {
                motorState = MotorState.MANUAL
                fromStateChange = true
            }
        }
        telemetryInstructions()
    }

    fun telemetryInstructions(): Unit {
        if (motorState == MotorState.MANUAL) {
            telemetry.addData("Power:", motorPower)
            telemetry.addData("Encoder Position:", motor.currentPosition)
            telemetry.addData("Motor Mode:", "${motor.mode}\n")

            telemetry.addLine("Use G1 triggers to move motor\n")
            telemetry.addLine("G1.A: Switch to Encoder Testing")
        }
        else if (motorState == MotorState.ENCODER) {
            telemetry.addData("Target Power:", "${motorPower}\n")
            telemetry.addData("Current Pos:", motor.currentPosition)
            telemetry.addData("Target Pos:", encoderTarget)
            telemetry.addData("Motor Mode:", "${motor.mode}")

            telemetry.addLine("Use G1 DPAD L & R to adjust motor power")
            telemetry.addLine("Use G1 RB & LB to adjust target position")
            telemetry.addLine("Use G1 B to reset target position")
            telemetry.addLine("Use G1 X to run to position\n")
            telemetry.addLine("Use G1 Y to switch to manual testing")
        }

        telemetry.update()
    }
}