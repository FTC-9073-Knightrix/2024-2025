package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ArmAndServo")
public class ArmAndServo extends OpMode {

    final double outtakeSpeed = -1.0;
    double servoRot = 0.0;

    //Automated time variables
    double current1 = Double.MAX_VALUE;
    double current2 = Double.MAX_VALUE;
    double current3 = Double.MAX_VALUE;
    double current4 = Double.MAX_VALUE;
    double current5 = Double.MAX_VALUE;

    DcMotor outtakeMotor;
    Servo armServo;
    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        telemetry.addData("Initialization","Done!");
    }

    @Override
    public void loop() {
        if (gamepad1.x) {

        }
        if (g1_dpad_left){
            launchServo.setPosition(-1);

        }
        if (g1_dpad_right){
            launchServo.setPosition(1);

        }

        if (g2_a){
            current2 = getRuntime();
            clawPos = 1.0;
        }

        if (getRuntime() > current2 + .75){
            current3 = getRuntime();
            servoRot = 0.601;
            current2 = Double.MAX_VALUE;
        }

        if (getRuntime() > current3 + .75){
            current4 = getRuntime();
            clawPos = 0.0;
            current3 = Double.MAX_VALUE;
        }

        if (getRuntime() > current4 + .75){
            current5 = getRuntime();
            servoRot = 0.0;
            current4 = Double.MAX_VALUE;
        }
    }
}