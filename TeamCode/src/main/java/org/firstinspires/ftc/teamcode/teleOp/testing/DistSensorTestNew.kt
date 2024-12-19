package org.firstinspires.ftc.teamcode.teleOp.testing

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp(name = "DistSensorTestNew")
class DistSensorTestNew : OpMode() {
    lateinit var motor: DcMotor
    lateinit var servo: Servo
    lateinit var distance: DistanceSensor
    var tooClose = false
    var tooFar = false

    override fun init() {
        motor = hardwareMap.dcMotor.get("motor") // 0
        servo = hardwareMap.servo.get("servo") // 0
        distance = hardwareMap.get(DistanceSensor::class.java, "distance") // i2c 1
        telemetry.addData("Init:", "Successful")
        telemetry.update()
    }

    override fun loop() {
        telemetry.addData("Distance (mm):", distance.mm())
        telemetry.addData("Too close:", tooClose)
        telemetry.addData("Too far:", tooFar)
        telemetry.update()

        tooClose = (distance.mm() < 4)
        tooFar = (distance.mm() > 5)

        if (tooClose) {
            motor.power = -0.2;
        } else if (tooFar) {
            motor.power = 0.2;
        } else {
            motor.stop()
        }
    }

    // Extension methods
    fun DcMotor.stop() {
        power = 0.0
    }

    fun DistanceSensor.mm(): Double {
        return getDistance(DistanceUnit.MM)
    }
}
