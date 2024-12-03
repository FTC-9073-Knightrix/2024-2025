package org.firstinspires.ftc.teamcode.teleOp.testing.andrewReference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "andrewTestingIntakeEncoder")
public class andrewTestingIntakeEncoder extends OpMode {
    DcMotor intake;
    double ticks = 1000.0;
    double newTarget;

    boolean isTurning = false;
    @Override
    public void init() {
        telemetry.addData("Initialization:", "is a success.");
        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "leftMotor");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            encoder(0.5);
        }
        if (gamepad1.b) {
            tracker();
        }

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
