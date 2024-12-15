package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "DistSensorTestNew")
class DistSensorTestNew : OpMode() {
    private lateinit var motor: DcMotor
    private lateinit var servo: Servo
    private lateinit var distance: DistanceSensor
    var tooClose: Boolean = false

    override fun init() {
        motor = hardwareMap.dcMotor.get("motor") // 0
        servo = hardwareMap.servo.get("servo") // 0
        distance = hardwareMap.get(DistanceSensor::class.java, "distance") // i2c 1
        telemetry.addData("Init:", "Successful")
        telemetry.update()
    }

    override fun loop() {
        telemetry.addData("Distance (cm):", distance.getDistance(DistanceUnit.CM))
        telemetry.addData("Distance (mm):", distance.getDistance(DistanceUnit.MM))
        telemetry.addData("Distance (in):", distance.getDistance(DistanceUnit.INCH))
        telemetry.addData("Too close:", tooClose)
        telemetry.update()

        tooClose = (distance.getDistance(DistanceUnit.CM) < 4)

        if (cm(distance) < 4) {
            motor.power = -0.5;
        } else if (cm(distance) > 10) {
            motor.power = 0.5;
        } else {
            stopMotor()
        }
    }

    private fun stopMotor() {
        motor.power = 0.0
    }

    private fun cm(distanceSensor: DistanceSensor): Double {
        return distanceSensor.getDistance(DistanceUnit.CM)
    }
}
