package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs

@TeleOp
class HorizSlideAndClawTest: OpMode() {
    lateinit var intakeClawServo: Servo
    lateinit var intakeTwistServo: Servo
    lateinit var intakeArmServo: Servo
    lateinit var horizLinearMotor: DcMotor
    lateinit var intakeDistanceSensor: DistanceSensor

    var intakeArmServoRot = 0.5
    override fun init() {
        // --------------------------------------- INITIALIZATION ---------------------------------------
        telemetry.addLine("Init starting")
        horizLinearMotor = hardwareMap.get(DcMotor::class.java, "horizLinearMotor")

        intakeClawServo = hardwareMap.get(Servo::class.java, "intakeClawServo")
        intakeTwistServo = hardwareMap.get(Servo::class.java, "intakeTwistServo")
        intakeArmServo = hardwareMap.get(Servo::class.java, "intakeArmServo")
        intakeClawServo.position = 1.0;

        intakeDistanceSensor = hardwareMap.get(DistanceSensor::class.java, "colorSensor")

        horizLinearMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT // let slide be moved around
        telemetry.addLine("Init success")
    }

    override fun loop() {
        telemetry.addData("Slide position", abs(horizLinearMotor.currentPosition))
        telemetry.addData("Power, direction", horizLinearMotor.power)
        telemetry.addData("Intake Arm position", intakeArmServoRot)
        telemetry.addData("Distance", intakeDistanceSensor.getDistance(DistanceUnit.MM)
        )
        horizLinearMotor.power = (-gamepad1.right_stick_y).toDouble()


        if (gamepad1.dpad_up) {
            intakeArmServoRot += 0.001
        }
        if (gamepad1.dpad_down) {
            intakeArmServoRot -= 0.001
        }
        intakeArmServo.position = intakeArmServoRot
    }
}