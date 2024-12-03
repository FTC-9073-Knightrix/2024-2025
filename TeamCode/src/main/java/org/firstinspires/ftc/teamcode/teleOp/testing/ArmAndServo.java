package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

// TODO Prevent overextension and under-extension of linearSlides
@TeleOp(name="ArmAndServo")
public class ArmAndServo extends OpMode {

    //Servo variables
    double armRot = 1.0;
    double basketRot = 0.0;
    double latchRot = 1.0;
    double clawRot = 0.0;

    //Motor variables
    int liftPosHoriz = 0;
    int liftPosAdjHoriz = 0;
    int liftPosVert = 0;
    int liftPosAdjVert = 0;
    double vertLinearPower = 0.0;
    double horizLinearPower = 0.0;
    double outtakePower = 0.0;

    boolean clickedX = false;

    //Automated time variables
    double current1 = Double.MAX_VALUE;
    double current2 = Double.MAX_VALUE;
    double current3 = Double.MAX_VALUE;
    double current4 = Double.MAX_VALUE;
    double current5 = Double.MAX_VALUE;

    // electronics
    public DcMotor outtakeMotor;
    public DcMotor vertLinearMotor;
    public DcMotor horizLinearMotor;

    public Servo armServo;
    public Servo clawServo;
    public Servo basketServo;
    public Servo latchServo;

    public TouchSensor vertSlideSensor; //slideLimit.isPressed(), assume encoder position is 8000
    public TouchSensor horizSlideSensor;


    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");

        outtakeMotor= hardwareMap.dcMotor.get("outtakeMotor"); // Motor Port 0
        vertLinearMotor = hardwareMap.dcMotor.get("vertLinearMotor"); // Motor Port 1
        horizLinearMotor = hardwareMap.dcMotor.get("horizLinearMotor"); // Motor Port 2

        armServo = hardwareMap.servo.get("armServo"); // Servo Port 0
        clawServo = hardwareMap.servo.get("clawServo"); // Servo Port 1
        basketServo = hardwareMap.servo.get("basketServo"); // Servo Port 2
        latchServo = hardwareMap.servo.get("latchServo"); // Servo Port 3

        vertSlideSensor = hardwareMap.touchSensor.get("vertSlideSensor"); // Digital 0
        horizSlideSensor = hardwareMap.touchSensor.get("horizSlideSensor"); // Digital 1


        vertLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Initialization","Done!");
    }

    @Override
    public void loop() {
        // ---------------------------------------INTAKE OUTTAKE SYSTEM ---------------------------------------
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
                armRot = 0.0;
            }

            if (getRuntime() > current1 + 1.25){
                // After 1.25 seconds from press -> Start outtake motor and shoot block out
                current2 = getRuntime();
                outtakePower = -1.0;
                current1 = Double.MAX_VALUE;
            }

            if (getRuntime() > current2 + 0.75) {
                // After 2.0 seconds from press -> Stop outtake motor and pull servo down, then close the latch
                outtakePower = 0.0;
                armRot = 1.0;
                latchRot = 0.0; // close latch
                current2 = Double.MAX_VALUE;
                clickedX = false;
            }
        }
        else {
            outtakePower = (gamepad2.right_trigger) - (gamepad2.left_trigger); // rt = intake, lt = outtake
            if (gamepad2.dpad_down){
                armRot += 0.005;
            }
            if (gamepad2.dpad_up){
                armRot -= 0.005;
            }
        }

        // ---------------------------------------BASKET LATCH DROP SYSTEM ---------------------------------------
        // (For both the basket and latch: 1.0 is open, 0.0 is close)
        if (gamepad2.b) { // Open servo rotate the basket to drop
            current3 = getRuntime();
            basketRot = 1.0;
        }
        if (getRuntime() > current3 + 0.75) { // Open latch servo - block drops from basket
            current3 = Double.MAX_VALUE;
            current4 = getRuntime();
            latchRot = 1.0;
        }
        if (getRuntime() > current4 + 0.75) { // After .75 sec - close latch servo
            current5 = getRuntime();
            current4 = Double.MAX_VALUE;
            latchRot = 0.0;
        }
        if (getRuntime() > current5 + 1.00) { // After 1 sec - return bucket
            current5 = Double.MAX_VALUE;
            basketRot = 0.0;
        }

        if (gamepad2.y) {
            basketRot = 0.0;
        }
        // --------------------------------------- CLAW SERVO SYSTEM ---------------------------------------
        // (0.0 is open, 1.0 is closed)
        if (gamepad2.left_bumper){
            clawRot = 0.0;
        }
        if (gamepad2.right_bumper){
            clawRot = 1.0;
        }

        // --------------------------------------- LINEAR SLIDES ---------------------------------------
        // Stop overextension and over retraction of vert linear motor
        // TODO CHANGE THE RIGHT_BUMPER AND LEFT_BUMPER ADJUSTMENT RESET TO SOME OTHER BUTTONS ON THE CONTROLLER
        liftPosVert = vertLinearMotor.getCurrentPosition() - liftPosAdjVert;

        if (vertSlideSensor.isPressed() || gamepad2.right_bumper) {
            liftPosAdjVert = vertLinearMotor.getCurrentPosition();
        }

        if (gamepad2.right_stick_y > 0 && !vertSlideSensor.isPressed()) {
            vertLinearPower = gamepad2.right_stick_y;
        } else if (gamepad2.right_stick_y < 0.0 && liftPosVert < 8000) {
            vertLinearPower = gamepad2.right_stick_y;
        } else { vertLinearPower = 0.0;}

        // Stop overextension and over retraction of horizontal linear motor
        liftPosHoriz = horizLinearMotor.getCurrentPosition() - liftPosAdjHoriz;

        if (horizSlideSensor.isPressed() || gamepad2.left_bumper) {
            liftPosAdjHoriz = horizLinearMotor.getCurrentPosition();
        }

        if (gamepad2.left_stick_y > 0 && !horizSlideSensor.isPressed()) {
            horizLinearPower = gamepad2.left_stick_y;
        } else if (gamepad2.left_stick_y < 0.0 && liftPosHoriz < 8000) {
            horizLinearPower = gamepad2.left_stick_y;
        } else { horizLinearPower = 0.0;}

        outtakeMotor.setPower(outtakePower);

        vertLinearMotor.setPower(vertLinearPower);
        horizLinearMotor.setPower(horizLinearPower);

        if (clawRot > 1) clawRot = 1;
        if (clawRot < 0) clawRot = 0;
        if (armRot > 1) armRot = 1;
        if (armRot < 0) armRot = 0;
        if (basketRot > 1) basketRot = 1;
        if (basketRot < 0) basketRot = 0;
        if (latchRot > 1) latchRot = 1;
        if (latchRot < 0) latchRot = 0;

        clawServo.setPosition(clawRot);
        armServo.setPosition(armRot);
        basketServo.setPosition(basketRot);
        latchServo.setPosition(latchRot);

        telemetry.addData("Clicked X", clickedX);
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("g2RStickY, g2LStickY", gamepad2.right_stick_y + ", " + gamepad2.left_stick_y);
        telemetry.addData("Vertical, Horizontal", liftPosVert + ", " + horizLinearMotor.getCurrentPosition());
        telemetry.update();
    }
}