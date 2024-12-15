package org.firstinspires.ftc.teamcode.teleOp.testing.andrewReference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Testing the Distance Sensor
// When distance from sensor is less than 10 cm -> prevent driver from spinning motor
@TeleOp(name = "DistanceSensorTest")
public class DistanceSensorTest extends OpMode {
    DcMotor motor;
    DistanceSensor distance;
    boolean tooClose = false;
    @Override
    public void init() {
        telemetry.addData("Init", "Successful");
        telemetry.update();

        motor = hardwareMap.get(DcMotor.class, "motor");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
    }

    @Override
    public void start() {
        telemetry.addData("Start", "Successful");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Distance", distance.getDistance(DistanceUnit.CM));
        telemetry.update();

        double power;
        power = gamepad1.right_trigger - gamepad1.left_trigger;
        tooClose = distance.getDistance(DistanceUnit.CM) < 10;

        if (tooClose && power > 0) {
            power = 0;
        }
        motor.setPower(power);
    }
}
