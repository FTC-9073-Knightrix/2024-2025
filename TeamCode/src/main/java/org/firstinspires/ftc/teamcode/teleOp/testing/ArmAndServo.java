package org.firstinspires.ftc.teamcode.teleOp.testing;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// TODO Prevent overextension and under-extension of linearSlides
// TODO Make a default position for the Arm to get it out the way of the bucket and off the floor
@TeleOp(name="ArmAndServo")
public class ArmAndServo extends OpMode {
    //Drive train speeds
    final double driveSpeed = 0.66;
    final double fastSpeed = 1.0;
    final double slowSpeed = 0.25;
    double finalSlowMode = 0.0;
    boolean slowMode = true;

    //Drivetrain hardware
    MecanumDrive mecanumDrive;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    //Servo variables
    double armRot = 0.5; // start the arm halfway
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
    double hangerPower = 0.0;

    boolean clickedX = false;
    boolean clickedA = false;
    boolean armInDefault = true;
    boolean startHanging = false;

    //Automated time variables
    double autoIntakeOuttakeCurrent1 = Double.MAX_VALUE;
    double autoIntakeOuttakeCurrent2 = Double.MAX_VALUE;
    double autoIntakeOuttakeCurrent3 = Double.MAX_VALUE;

    double basketDropCurrent1 = Double.MAX_VALUE;
    double basketDropCurrent2 = Double.MAX_VALUE;
    double current6 = Double.MAX_VALUE;

    // Electronics
    public DcMotor outtakeMotor;
    public DcMotor vertLinearMotor;
    public DcMotor horizLinearMotor;
    public DcMotor hangerMotor;

    public Servo armServo;
    public Servo clawServo;
    public Servo basketServo;
    public Servo latchServo;

    public TouchSensor vertSlideSensor;
    public TouchSensor horizSlideSensor;
    public TouchSensor hangerSensor1;
    public TouchSensor hangerSensor2;

    //Create the gyroscope
    public IMU imu;

    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;


    @Override
    public void init() {
        telemetry.addData("Initialization","Starting...");

        outtakeMotor= hardwareMap.dcMotor.get("intakeMotor"); // Expansion Hub Motor 0
        vertLinearMotor = hardwareMap.dcMotor.get("vertLinearMotor"); // Expansion Hub Motor 1
        horizLinearMotor = hardwareMap.dcMotor.get("horizLinearMotor"); // Expansion Hub Motor 2
        hangerMotor = hardwareMap.dcMotor.get("hangerMotor"); // Expansion Hub Motor 3

        armServo = hardwareMap.servo.get("armServo"); // Control Hub Servo 0
        clawServo = hardwareMap.servo.get("clawServo"); // Control Hub Servo  1
        basketServo = hardwareMap.servo.get("basketServo"); // Control Hub Servo 2
        latchServo = hardwareMap.servo.get("latchServo"); // Control Hub Servo 3

        vertSlideSensor = hardwareMap.touchSensor.get("vertSlideSensor"); // Control Hub Digital 0
        horizSlideSensor = hardwareMap.touchSensor.get("horizSlideSensor"); // Control Hub Digital 1
        hangerSensor1 = hardwareMap.touchSensor.get("hangerSensor1"); // Control Hub Digital 2
        hangerSensor2 = hardwareMap.touchSensor.get("hangerSensor2"); // Control Hub Digital 3

        leftFront = new Motor(hardwareMap, "LF"); // Control Hub Motor Port 0
        leftBack = new Motor(hardwareMap, "LB"); // Control Hub Motor Port 1
        rightFront = new Motor(hardwareMap, "RF"); // Control Hub Motor Port 2
        rightBack = new Motor(hardwareMap, "RB"); // Control Hub Motor Port 3

        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        vertLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Initialization","Done!");
    }

    @Override
    public void loop() {
        // ---------------------------------------MECANUM DRIVE SYSTEM ---------------------------------------
        //Setting boolean hold
        if(gamepad1.right_bumper) {
            //Slowmode
            finalSlowMode = slowSpeed;

        } else if (gamepad1.left_bumper) {
            //Fastmode
            finalSlowMode = fastSpeed;
        } else {
            //Regular
            finalSlowMode = driveSpeed;
        }

        if (gamepad1.y){
            imu.resetYaw();
        }

        orientation = imu.getRobotYawPitchRollAngles();
        mecanumDrive.driveFieldCentric(gamepad1.left_stick_x * finalSlowMode, -gamepad1.left_stick_y * finalSlowMode, gamepad1.right_stick_x * finalSlowMode, orientation.getYaw(AngleUnit.DEGREES));

        // ---------------------------------------INTAKE OUTTAKE SYSTEM ---------------------------------------
        // TODO PULL THE HORIZONTAL SLIDE BACK AUTOMATICALLY
        if (gamepad2.a){
            clickedA = true;
        }

        if (clickedA) {
            // outtakeServo (-1 is intake, 1 is outtake)
            // armServo (0 is up, 1 is down)
            if (gamepad2.a) {
                // Gamepad 1 x pressed -> Stop outtake motor and pull servo up
                outtakePower = 0.0;
                latchRot = 1.0;
                autoIntakeOuttakeCurrent1 = getRuntime();
                armRot = 0.0;
            }

            if (getRuntime() > autoIntakeOuttakeCurrent1 + 1.25) {
                // After 1.25 seconds from press -> Start outtake motor and shoot block out
                autoIntakeOuttakeCurrent2 = getRuntime();
                outtakePower = -1.0;
                autoIntakeOuttakeCurrent1 = Double.MAX_VALUE;
            }

            if (getRuntime() > autoIntakeOuttakeCurrent2 + 0.75) {
                // After 2.0 seconds from press -> Stop outtake motor and pull servo down to halfway, then close the latch
                autoIntakeOuttakeCurrent2 = Double.MAX_VALUE;
                autoIntakeOuttakeCurrent3 = getRuntime();
                outtakePower = 0.0;
                armRot = 0.7;
            }
            // After 2.5 seconds from press -> Close latch
            if (getRuntime() > autoIntakeOuttakeCurrent3 + 0.5) {
                autoIntakeOuttakeCurrent3 = Double.MAX_VALUE;
                latchRot = 0.0;
                clickedA = false;
            }
        }
        else {
            outtakePower = (gamepad2.right_trigger) - (gamepad2.left_trigger); // rt = intake, lt = outtake

            if (gamepad2.x) {
                clickedX = true;
            }
            if (clickedX) {
                if (gamepad2.x) {
                    current6 = getRuntime();
                    if (armInDefault) {
                        armRot = 0.015;
                    } else {
                        armRot = 0.5;
                    }

                }
                if (getRuntime() > current6 + 0.5) {
                    armInDefault = !armInDefault;
                    current6 = Double.MAX_VALUE;
                    clickedX = false;
                }
            } else {
                // Moving arm with dpad
                armRot = armRot + ((gamepad2.dpad_down ? 1 : 0) + (gamepad2.dpad_up ? -1 : 0)) * 0.01;
            }
        }

        // ---------------------------------------BASKET LATCH DROP SYSTEM ---------------------------------------
        // (For both the basket and latch: 1.0 is open, 0.0 is close)
        if (gamepad2.right_bumper) { // Open servo rotate the basket to drop
            basketRot = 1.0;
        }
        if (gamepad2.left_bumper) { // Open latch servo to drop
            basketDropCurrent1 = getRuntime();
            latchRot = 1.0;
        }
        if (getRuntime() > basketDropCurrent1 + 0.75) { // After .75 sec - close latch servo
            basketDropCurrent2 = getRuntime();
            basketDropCurrent1 = Double.MAX_VALUE;
            latchRot = 0.0;
        }
        if (getRuntime() > basketDropCurrent2 + 1.00) { // After 1 sec - return bucket
            basketRot = Range.clip(basketRot, 0.0, 1.0) - 0.02;
        }
        if (getRuntime() > basketDropCurrent2 + 3.00) { // After 2 sec - Max the current after the bucket returns
            basketDropCurrent2 = Double.MAX_VALUE;
        }

        if (gamepad2.right_stick_button) {
            basketRot = 0.0;
        }
        // --------------------------------------- CLAW SERVO SYSTEM ---------------------------------------
        // (0.2 is closed, 1.0 is open)
        if (gamepad1.b){
            clawRot = 0.2;
        }
        if (gamepad1.a){
            clawRot = 1.0;
        }

        // --------------------------------------- LINEAR SLIDES ---------------------------------------
        // Stop overextension and over retraction of vert linear motor
        liftPosVert = -(vertLinearMotor.getCurrentPosition() - liftPosAdjVert);

        if (vertSlideSensor.isPressed() || gamepad2.dpad_right) { // if slide sensor touched or manual adjustment button pressed
            liftPosAdjVert = vertLinearMotor.getCurrentPosition();
        }

        if (gamepad2.left_stick_y > 0 && !vertSlideSensor.isPressed()) {
            vertLinearPower = -gamepad2.left_stick_y;
        } else if (gamepad2.left_stick_y < 0 && liftPosVert < 10000) {
            vertLinearPower = -gamepad2.left_stick_y;
        } else { vertLinearPower = 0.0;}

        // Stop overextension and over retraction of horizontal linear motor
        liftPosHoriz = -(horizLinearMotor.getCurrentPosition() - liftPosAdjHoriz);

        if (horizSlideSensor.isPressed() || gamepad2.dpad_left) {  // if slide sensor touched or manual adjustment button pressed
            liftPosAdjHoriz = horizLinearMotor.getCurrentPosition();
        }

        if (gamepad2.right_stick_y > 0

        ) {
            horizLinearPower = -gamepad2.right_stick_y * 0.5;
        } else if (gamepad2.right_stick_y < 0.0 && liftPosHoriz < 8000) {
            horizLinearPower = -gamepad2.right_stick_y * 0.5;
        } else { horizLinearPower = 0.0;}

        // --------------------------------------- LEAD SCREW ---------------------------------------
        //Only allow hanger to move if not at the extrema, and the limits are not touched.
        if (gamepad1.dpad_up){
            startHanging = true;
        }
        if (gamepad1.dpad_down){
            startHanging = false;
            if (!hangerSensor1.isPressed()){
                hangerPower = -1.0;
            }
        }

        if (startHanging && !hangerSensor2.isPressed()) { hangerPower = 1.0; }
        else if (gamepad1.dpad_down && !hangerSensor1.isPressed()) { hangerPower = -1.0; }
        else { hangerPower = 0.0; }

        outtakeMotor.setPower(outtakePower);
        vertLinearMotor.setPower(-vertLinearPower);
        horizLinearMotor.setPower(horizLinearPower);
        hangerMotor.setPower(hangerPower);

        if (clawRot > 1) clawRot = 1;
        if (clawRot < 0.2) clawRot = 0.2;
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

        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Clicked X", clickedX);
        telemetry.addData("Magnet Pressed", vertSlideSensor.isPressed());
        telemetry.addData("g2RStickY, g2LStickY", gamepad2.right_stick_y + ", " + gamepad2.left_stick_y);
        telemetry.addData("Vertical, True", liftPosVert + ", " + vertLinearMotor.getCurrentPosition());
        telemetry.addData("Horizontal, True", liftPosHoriz + ", " + vertLinearMotor.getCurrentPosition());
        telemetry.addData("Gyro: ", "Yaw: " + orientation.getYaw(AngleUnit.DEGREES) + "Roll: " + orientation.getRoll(AngleUnit.DEGREES) + "Pitch: " + orientation.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Slowmode: ", finalSlowMode);
        telemetry.addData("Arm Pos: ", armRot);
        telemetry.update();
    }
}