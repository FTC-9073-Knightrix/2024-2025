package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

// TODO Change outtake motor to a continuous servo
// TODO Prevent overextension and underextension of linearSlides
@TeleOp(name="ArmAndServo")
public class ArmAndServo extends OpMode {

    double outtakePower = 0.0;
    double armServoRot = 1.0;
    double basketRot = 0.0;
    boolean clickedX = false;
    boolean claw = false;
    double clawRot = 0.0;

    TouchSensor slideLimit; //slideLimit.isPressed(), assume encoder position is 8000

    //Automated time variables
    double current1 = Double.MAX_VALUE;
    double current2 = Double.MAX_VALUE;
    double current3 = Double.MAX_VALUE;
    double current4 = Double.MAX_VALUE;
//    public DcMotor outtakeMotor; Switched to a servo
    public DcMotor vertLinearMotor;
    public DcMotor horizLinearMotor;

    public CRServo outtakeServo; // Goes -1 to 1
    public Servo armServo;
    public Servo clawServo;
    public Servo basketServo;

    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");
        telemetry.addData("Initialization","Done!");

        vertLinearMotor = hardwareMap.dcMotor.get("vertLinearMotor"); // Motor Port 0
        horizLinearMotor = hardwareMap.dcMotor.get("horizLinearMotor"); // Motor Port 1

        armServo = hardwareMap.servo.get("rotateServo"); // Servo Port 0
        clawServo = hardwareMap.servo.get("clawServo"); // Servo Port 1
        basketServo = hardwareMap.servo.get("basketServo"); // Servo Port 2
        outtakeServo = hardwareMap.crservo.get("outtakeServo"); // Servo Port 3

        vertLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (gamepad2.x){
            clickedX = true;
        }

        if (clickedX) {
            // outtakeServo (-1 is intake, 1 is outtake)
            // armServo (0 is up, 1 is down)
            if (gamepad2.x) {
                // Gamepad 1 x pressed -> Stop outtake motor and pull servo up
                outtakePower = 0.0;
                current1 = getRuntime();
                armServoRot = 0.0;
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
                armServoRot = 1.0;
                current2 = Double.MAX_VALUE;
                clickedX = false;
            }
        }
        else {
            outtakePower = (gamepad2.right_trigger) - (gamepad2.left_trigger);
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

        // Stop Linear Overextension & Over-retraction
//        if (vertLinearMotor.getCurrentPosition() >= 1.0) {
//
//        }

        outtakeServo.setPower(outtakePower);

        vertLinearMotor.setPower(gamepad2.right_stick_y);
        horizLinearMotor.setPower(gamepad2.left_stick_y);

        clawServo.setPosition(clawRot);
        armServo.setPosition(armServoRot);
        basketServo.setPosition(basketRot);

        telemetry.addData("Clicked", clickedX);
        telemetry.addData("Claw (open)", claw);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Current1, Current2, Current3", current1 + ", " + current2 + ", " + current3);
        telemetry.addData("g2RStickY, g2LStickY", gamepad2.right_stick_y + ", " + gamepad2.left_stick_y);
        telemetry.update();
    }
}