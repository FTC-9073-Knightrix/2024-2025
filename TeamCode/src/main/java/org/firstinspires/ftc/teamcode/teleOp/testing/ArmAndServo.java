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
    public DcMotor outtakeMotor;
    public Servo armServo;
    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");
        telemetry.addData("Initialization","Done!");
        outtakeMotor = hardwareMap.dcMotor.get("outtakeMotor");
        armServo = hardwareMap.servo.get("");
    }

    @Override
    public void loop() {

        double power = gamepad1.left_trigger - gamepad1.right_trigger;
        outtakeMotor.setPower(power);
        telemetry.addData("Power:", power);

        if (gamepad1.x) { // Gamepad 1 x pressed -> Stop outtake motor and pull servo up
            telemetry.addData("Gamepad 1", "x");
            current1 = getRuntime();

            servoRot = 1.0;
            outtakeMotor.setPower(0);

            armServo.setPosition(servoRot);
        }

        if (getRuntime() > current1 + .75){ // After .75 seconds from press -> Start outtake motor and shoot block out
            current2 = getRuntime();
            outtakeMotor.setPower(1);
            current1 = Double.MAX_VALUE;
        }

        if (getRuntime() > current2 + .75){ // After 1.5 seconds from press -> Stop outtake motor and pull servo down
            current3 = getRuntime();
            outtakeMotor.setPower(0);
            servoRot = 0.0;

            armServo.setPosition(servoRot);
            current2 = Double.MAX_VALUE;
        }

        telemetry.update();
    }
}