package org.firstinspires.ftc.teamcode.teleOp.testing.andrewReference;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.util.Date;
@TeleOp(name="andrewTestingIntake")
public class andrewTestingIntake extends OpMode {
    DcMotor intake;
    DcMotorEx intake2; // diff type of motor with setVelocity(ticks/s) method

    @Override
    public void init() {
        // Happens once on initialization
        telemetry.addData("Initialization", "was a success at " + new Date()); // Sends this text to the driver station
        telemetry.update();
        intake = hardwareMap.get(DcMotor.class, "intake");
//        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Encoder ensures that motor is at consistent power
//        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Without encoder to track power, position
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Resets encoder ticks to 0
    }

    @Override
    public void init_loop() {
        // Happens repeatedly during init
    }

    @Override
    public void start() {
        // Happens only once on start
        telemetry.addData("Start", "was a success at " + new Date());
        telemetry.update();
    }

    @Override
    public void loop() {
        // Happens repeatedly after program has been started

        // Left Trigger: Accelerate | Right Trigger: Brake
        double tgtPower = gamepad1.left_trigger - gamepad1.right_trigger; // Calculates the motor power output
        intake.setPower(tgtPower);
        telemetry.addData("Target Power: ", tgtPower);
        telemetry.addData("Actual Power: ", intake.getPower());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Happens once after stop
        telemetry.addData("Stop", "was a success at " + new Date());
        telemetry.update();
    }
}
