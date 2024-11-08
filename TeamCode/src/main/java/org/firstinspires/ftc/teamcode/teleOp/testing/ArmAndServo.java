package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ArmAndServo")
public class ArmAndServo extends OpMode {

    double outtakePower = -1.0;
    double servoRot = 1.0;
    double basketRot = 0.0;
    boolean clicked = false;
    boolean claw = false;
    double clawRot = 0.0;

    //Automated time variables
    double current1 = Double.MAX_VALUE;
    double current2 = Double.MAX_VALUE;
    double current3 = Double.MAX_VALUE;
    double current4 = Double.MAX_VALUE;
    public DcMotor outtakeMotor;
    public DcMotor linearMotor;
    public Servo armServo;
    public Servo clawServo;
    public Servo basketServo;

    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");
        telemetry.addData("Initialization","Done!");
        outtakeMotor = hardwareMap.dcMotor.get("outtakeMotor");
        linearMotor = hardwareMap.dcMotor.get("linearMotor");
        armServo = hardwareMap.servo.get("rotateServo");
        clawServo = hardwareMap.servo.get("clawServo");
        basketServo = hardwareMap.servo.get("basketServo");

        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad2.x){
            clicked = true;
        }

        if (clicked) {
            // outtakeMotor (1 is outtake, -1 is intake)
            // armServo (0 is up, 1 is down)
            if (gamepad2.x) {
                // Gamepad 1 x pressed -> Stop outtake motor and pull servo up
                outtakePower = 0;
                current1 = getRuntime();
                servoRot = 0.0;
            }

            if (getRuntime() > current1 + .75){
                // After .75 seconds from press -> Start outtake motor and shoot block out
                current2 = getRuntime();
                outtakePower = 1.0;
                current1 = Double.MAX_VALUE;
            }

            if (getRuntime() > current2 + 1.5) {
                // After 1.5 seconds from press -> Stop outtake motor and pull servo down
                outtakePower = 0.0;
                servoRot = 1.0;
                current2 = Double.MAX_VALUE;
                clicked = false;
            }
        }
        else {
            outtakePower = gamepad2.right_trigger - gamepad2.left_trigger;
        }

        // Basket Drop Servo System
        if (gamepad2.b) { // Open servo - drop the basket
            current3 = getRuntime();
            basketRot = 1.0;
        }
        if (getRuntime() > current3 + 0.75) { // After .75 sec - bring the basket back
            basketRot = 0.0;
            current3 = Double.MAX_VALUE;
        }

        // Claw Servo System - (0 is open, 1 is closed)
        if (gamepad2.left_bumper){
            clawRot = 0.0;
        }
        if (gamepad2.right_bumper){
            clawRot = 1.0;
        }

        linearMotor.setPower(gamepad2.right_stick_y);

        telemetry.addData("Clicked", clicked);
        telemetry.addData("Claw Open", claw);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Current1, Current2, Current3", current1 + ", " + current2 + ", " + current3);

        clawServo.setPosition(clawRot);
        outtakeMotor.setPower(outtakePower);
        armServo.setPosition(servoRot);
        basketServo.setPosition(basketRot);
        telemetry.update();
    }
}