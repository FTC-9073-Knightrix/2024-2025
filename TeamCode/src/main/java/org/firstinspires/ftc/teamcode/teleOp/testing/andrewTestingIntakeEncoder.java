package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "andrewTestingIntakeEncoder")
public class andrewTestingIntakeEncoder extends OpMode {
    DcMotor intake;
    double ticks = 1000.0;
    double newTarget;

    DistanceSensor sensorDistance;

    boolean isTurning = false;
    @Override
    public void init() {
        telemetry.addData("Initialization", "is a success.");
        telemetry.update();

        intake = hardwareMap.get(DcMotor.class, "leftMotor");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorDistance = hardwareMap.get(DistanceSensor.class, "distance");
    }

    @Override
    public void start() {
        telemetry.addData("Start", "is a success.");
        telemetry.update();
    }
    @Override
    public void loop() {
        telemetry.addData("deviceName", sensorDistance.getDeviceName());
//        telemetry.addData("range", sensorDistance.getDistance(DistanceUnit.MM) + " mm");
        telemetry.addData("range", sensorDistance.getDistance(DistanceUnit.CM) + " cm");
//        telemetry.addData("range", sensorDistance.getDistance(DistanceUnit.METER) + " m");
//        telemetry.addData("range", sensorDistance.getDistance(DistanceUnit.INCH) + " in");
        if (gamepad1.a) {
            encoder(0.5);
        }
        if (gamepad1.b) {
            tracker();
        }
        telemetry.update();
    }

    public void encoder(double turnage) {
        newTarget = ticks*turnage;
        intake.setTargetPosition((int) newTarget);
        intake.setPower(0.3);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION); // motor maintains position even if force is applied to disturb position
    }

    public void tracker() {
        intake.setTargetPosition(0);
        intake.setPower(0.8);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
