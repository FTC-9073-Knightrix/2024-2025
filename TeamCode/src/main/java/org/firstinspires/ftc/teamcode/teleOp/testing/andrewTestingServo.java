package org.firstinspires.ftc.teamcode.teleOp.testing;

import static org.firstinspires.ftc.teamcode.teleOp.mecDrive.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="andrewTestingServo")
public class andrewTestingServo extends OpMode {
    public CRServo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class, "servo");
    }

    @Override
    public void loop() {
        while (gamepad1.right_bumper) {
            servo.setPower(-1);
        }
        while (gamepad1.left_bumper) {
            servo.setPower(1);
        }
        servo.setPower(0);
    }
}
