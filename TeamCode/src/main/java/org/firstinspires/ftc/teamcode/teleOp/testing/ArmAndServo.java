package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

// TODO Prevent overextension and under-extension of linearSlides
@TeleOp(name="ArmAndServo")
public class ArmAndServo extends OpMode {

    double outtakePower = 0.0;
    double armServoRot = 1.0;
    double basketRot = 0.0;

    int liftPosHoriz = 0;
    int liftPosAdjHoriz = 0;
    int liftPosVert = 0;
    int liftPosAdjVert = 0;
    double vertLinearPower = 0;
    double horizLinearPower = 0;
    boolean clickedX = false;
    boolean claw = false;
    double clawRot = 0.0;

    TouchSensor vertSlideLimit; //slideLimit.isPressed(), assume encoder position is 8000
    TouchSensor horizSlideLimit;

    //Automated time variables
    double current1 = Double.MAX_VALUE;
    double current2 = Double.MAX_VALUE;
    double current3 = Double.MAX_VALUE;
    double current4 = Double.MAX_VALUE;
    DcMotor outtakeMotor; //Switched to a servo
    public DcMotor vertLinearMotor;
    public DcMotor horizLinearMotor;

    public Servo armServo;
    public Servo clawServo;
    public Servo basketServo;


    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");
        telemetry.addData("Initialization","Done!");

        outtakeMotor= hardwareMap.dcMotor.get("outtakeMotor"); // Motor Port 0
        vertLinearMotor = hardwareMap.dcMotor.get("vertLinearMotor"); // Motor Port 1
        horizLinearMotor = hardwareMap.dcMotor.get("horizLinearMotor"); // Motor Port 2

        armServo = hardwareMap.servo.get("armServo"); // Servo Port 0
        clawServo = hardwareMap.servo.get("clawServo"); // Servo Port 1
        basketServo = hardwareMap.servo.get("basketServo"); // Servo Port 2

        vertSlideLimit = hardwareMap.touchSensor.get("vertSlideLimit"); // Digital 0
        horizSlideLimit = hardwareMap.touchSensor.get("horizSlideLimit"); // Digital 1

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
                outtakePower = -1.0;
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
            outtakePower = (gamepad2.right_trigger) - (gamepad2.left_trigger); // rt = intake, lt = outtake
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

        // Stop overextension and over retraction of vert linear motor
        liftPosVert = vertLinearMotor.getCurrentPosition() - liftPosAdjVert;

        if (vertSlideLimit.isPressed() || gamepad2.right_bumper) {
            liftPosAdjVert = vertLinearMotor.getCurrentPosition();
        }

        if (gamepad2.right_stick_y > 0 && !vertSlideLimit.isPressed()) {
            vertLinearPower = gamepad2.right_stick_y;
        } else if (gamepad2.right_stick_y < 0.0 && liftPosVert < 8000) {
            vertLinearPower = gamepad2.right_stick_y;
        } else { vertLinearPower = 0.0;}

        // Stop overextension and over retraction of horizontal linear motor
        liftPosHoriz = horizLinearMotor.getCurrentPosition() - liftPosAdjHoriz;

        if (horizSlideLimit.isPressed() || gamepad2.left_bumper) {
            liftPosAdjHoriz = horizLinearMotor.getCurrentPosition();
        }

        if (gamepad2.left_stick_y > 0 && !horizSlideLimit.isPressed()) {
            horizLinearPower = gamepad2.left_stick_y;
        } else if (gamepad2.left_stick_y < 0.0 && liftPosHoriz < 8000) {
            horizLinearPower = gamepad2.left_stick_y;
        } else { horizLinearPower = 0.0;}

        outtakeMotor.setPower(outtakePower);

        vertLinearMotor.setPower(vertLinearPower);
        horizLinearMotor.setPower(horizLinearPower);

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