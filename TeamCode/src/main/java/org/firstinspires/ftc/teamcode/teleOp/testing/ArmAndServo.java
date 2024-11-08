package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="ArmAndServo")
public class ArmAndServo extends OpMode {

    final double outtakeSpeed = -1.0;
    double servoRot = 0.0;

    //Automated time variables
    double current1 = Double.MAX_VALUE;
    double current2 = Double.MAX_VALUE;
    double current3 = Double.MAX_VALUE;

    DcMotor outtakeMotor;
    Servo armServo;

    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");
        outtakeMotor = hardwareMap.get(DcMotor.class, "outtakeMotor");
        armServo = hardwareMap.get(Servo.class, "armServo");
        telemetry.addData("Initialization","Done!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Gamepad 1 x pressed -> Stop outtake motor and pull servo up
        if (gamepad1.x) {
            telemetry.addData("Gamepad 1", "x pressed");
            current1 = getRuntime();

            servoRot = 1.0;
            outtakeMotor.setPower(0);

            armServo.setPosition(servoRot);
        }

        // After .75 seconds from press -> Start outtake motor and shoot block out
        if (getRuntime() > current1 + .75){
            current2 = getRuntime();
            outtakeMotor.setPower(1);
            current1 = Double.MAX_VALUE;
        }

        // After 1.5 seconds from press -> Stop outtake motor and pull servo down
        if (getRuntime() > current2 + .75){
            current3 = getRuntime();
            outtakeMotor.setPower(0);
            servoRot = 0.0;

            armServo.setPosition(servoRot);
            current2 = Double.MAX_VALUE;
        }

        telemetry.update();
    }

}
