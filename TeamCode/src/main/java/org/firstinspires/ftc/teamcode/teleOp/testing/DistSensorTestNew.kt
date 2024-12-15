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
        telemetry.addData("Distance (cm):", distance.getDistance(DistanceUnit.CM))
//        telemetry.addData("Distance (mm):", distance.getDistance(DistanceUnit.MM))
//        telemetry.addData("Distance (in):", distance.getDistance(DistanceUnit.INCH))
        telemetry.addData("Too close:", tooClose)
        telemetry.addData("Too far:", tooFar)
        telemetry.update()

        tooClose = (distance.cm() < 4)
        tooFar = (distance.cm() > 5)

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

    fun DistanceSensor.cm(): Double {
        return getDistance(DistanceUnit.CM)
    }
}
