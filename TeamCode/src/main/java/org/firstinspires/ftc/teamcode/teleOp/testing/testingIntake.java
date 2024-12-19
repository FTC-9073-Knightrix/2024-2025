package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test")
@Disabled // Unused Opmode
public class testingIntake extends OpMode {
    public DcMotor intake;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("motor");
    }

    @Override
    public void loop() {
        double power = gamepad1.left_trigger - gamepad1.right_trigger;
        intake.setPower(power);
        telemetry.addData("Power:", power);
    }
}
