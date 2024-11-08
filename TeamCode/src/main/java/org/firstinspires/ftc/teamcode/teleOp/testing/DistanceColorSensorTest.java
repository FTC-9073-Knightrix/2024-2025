package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="DistanceColorSensorTest")
public class DistanceColorSensorTest extends OpMode {
    DcMotor outtakeMotor;
    DistanceSensor distance;
    ColorSensor color;
    boolean tooClose = false;
    @Override
    public void init() {
        telemetry.addData("Initialization", "Running");

        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        color = hardwareMap.get(ColorSensor.class, "color");
        telemetry.addData("Initialization", "Successful");
        telemetry.update();
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
        outtakeMotor.setPower(power);
    }
}
