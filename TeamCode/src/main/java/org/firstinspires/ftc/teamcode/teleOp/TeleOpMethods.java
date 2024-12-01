package org.firstinspires.ftc.teamcode.teleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Config
public abstract class TeleOpMethods extends OpMode {
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

    double armDownPos = 0.68;
    double armDefaultPos = 0.5;

    // Gamepad Conditionals
    boolean clickedA = false;
    boolean startHanging = false;
    enum g2Bumpers {
        NONE,
        LEFT,
        RIGHT
    }
    g2Bumpers currentG2BumpersPressed = g2Bumpers.NONE;

    //Automated time variables
    double autoIntakeOuttakeCurrent1 = Double.MAX_VALUE;
    double autoIntakeOuttakeCurrent2 = Double.MAX_VALUE;
    double autoIntakeOuttakeCurrent3 = Double.MAX_VALUE;

    double basketDropCurrent1 = Double.MAX_VALUE;
    double basketDropCurrent2 = Double.MAX_VALUE;
    double basketDropCurrent3 = Double.MAX_VALUE;

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

        outtakeMotor= hardwareMap.dcMotor.get("outtakeMotor"); // Expansion Hub Motor 0
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

        // Reverse back wheel directions
        rightFront.setInverted(true);
        rightBack.setInverted(true);

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

    public void runMecanumDrive(){
        // ---------------------------------------MECANUM DRIVE ---------------------------------------
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


        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x * .5;

        if (gamepad1.y) {
            imu.resetYaw();
        }

        orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        leftFront.set(frontLeftPower * finalSlowMode);
        leftBack.set(backLeftPower * finalSlowMode);
        rightFront.set(frontRightPower * finalSlowMode);
        rightBack.set(backRightPower * finalSlowMode);
    }

    public void intakeOuttakeSystem() {
        // ---------------------------------------INTAKE OUTTAKE SYSTEM ---------------------------------------
        if (gamepad2.a){
            clickedA = true;
        }

        if (clickedA) {
            // outtakeServo (1 is intake, -1 is outtake)
            // armServo (0 is up, 1 is down)
            if (gamepad2.a) {
                // Gamepad 1 x pressed -> Stop outtake motor and pull servo up
                outtakePower = 0.0;
                horizLinearPower = -0.75;
                latchRot = 1.0;
                autoIntakeOuttakeCurrent1 = getRuntime();
                armRot = 0.05;
            }

            if (getRuntime() > autoIntakeOuttakeCurrent1 + 0.1) {
                // After 1.25 seconds from press -> Start outtake motor and shoot block out
                autoIntakeOuttakeCurrent2 = getRuntime();
                horizLinearPower = 0.0;
                outtakePower = -1.0;
                autoIntakeOuttakeCurrent1 = Double.MAX_VALUE;
            }

            if (getRuntime() > autoIntakeOuttakeCurrent2 + 0.75) {
                // After 2.0 seconds from press -> Stop outtake motor and pull servo down to halfway, then close the latch
                autoIntakeOuttakeCurrent2 = Double.MAX_VALUE;
                autoIntakeOuttakeCurrent3 = getRuntime();
                outtakePower = 0.0;
                armRot = armDefaultPos;
            }
            // After 2.5 seconds from press -> Close latch
            if (getRuntime() > autoIntakeOuttakeCurrent3 + 0.3) {
                autoIntakeOuttakeCurrent3 = Double.MAX_VALUE;
                latchRot = 0.0;
                clickedA = false;
            }
        }
        else {
            outtakePower = (gamepad2.right_trigger) - (gamepad2.left_trigger); // rt = intake, lt = outtake

            if (gamepad2.x) armRot = armDownPos; // Arm Down
            if (gamepad2.y) armRot = armDefaultPos; // Arm Default
            if (gamepad2.b) armRot = 0.0; // Arm at Bucket
            // Moving arm with dpad
            armRot = armRot + ((gamepad2.dpad_down ? 1 : 0) + (gamepad2.dpad_up ? -1 : 0)) * 0.015;
        }
    }

    public void basketSystem() {
        // ---------------------------------------BASKET LATCH DROP SYSTEM ---------------------------------------
        // (For both the basket and latch: 1.0 is open, 0.0 is close)
        if (clickedA) return; //PREVENTS BASKET MOVING WHILE IN INTAKE OUTTAKE SYSTEM

        if (gamepad2.right_bumper && currentG2BumpersPressed == g2Bumpers.NONE) {
            currentG2BumpersPressed = g2Bumpers.RIGHT;
        }
        else if (gamepad2.left_bumper && currentG2BumpersPressed == g2Bumpers.NONE) {
            currentG2BumpersPressed = g2Bumpers.LEFT;
        }

        switch (currentG2BumpersPressed) {
            case RIGHT:
                if (gamepad2.right_bumper) { // Put basket servo in default position
                    basketDropCurrent1 = getRuntime();
                    basketRot = Range.clip(basketRot, 0.0, 0.6) + 0.05;
                }
                if (getRuntime() > basketDropCurrent1 + 0.1) { // After 0.1 sec - rotate basket fully on release
                    basketRot = Range.clip(basketRot, 0.0, 1.0) + 0.03;
                }
                if (getRuntime() > basketDropCurrent1 + 0.1 && basketRot >= 1.0){
                    basketDropCurrent1 = Double.MAX_VALUE;
                    currentG2BumpersPressed = g2Bumpers.NONE;
                }
            case LEFT:
                if (gamepad2.left_bumper) { // Open latch servo to drop
                    basketDropCurrent2 = getRuntime();
                    latchRot = 1.0;
                }
                if (getRuntime() > basketDropCurrent2 + 0.75) { // After .75 sec - close latch servo
                    basketDropCurrent3 = getRuntime();
                    basketDropCurrent2 = Double.MAX_VALUE;
                    latchRot = 0.0;
                }
                if (getRuntime() > basketDropCurrent3 + 0.1) { // After 0.25 sec - return bucket
                    basketRot = Range.clip(basketRot, 0.0, 1.0) - 0.03;
                }
                if (getRuntime() > basketDropCurrent3 + 0.1 && basketRot <= 0.0) { // After 2 sec - Max the current after the bucket returns
                    basketDropCurrent3 = Double.MAX_VALUE;
                    currentG2BumpersPressed = g2Bumpers.NONE;
                }
        }
//        if (clickedRB) {
//            if (gamepad2.right_bumper) { // Put basket servo in default position
//                basketDropCurrent1 = getRuntime();
//                basketRot = Range.clip(basketRot, 0.0, 0.6) + 0.05;
//            }
//            if (getRuntime() > basketDropCurrent1 + 0.1) { // After 0.1 sec - rotate basket fully on release
//                basketRot = Range.clip(basketRot, 0.0, 1.0) + 0.03;
//            }
//            if (getRuntime() > basketDropCurrent1 + 0.1 && basketRot >= 1.0){
//                basketDropCurrent1 = Double.MAX_VALUE;
//                clickedRB = false;
//            }
//        } else {
//            if (gamepad2.left_bumper) { // Open latch servo to drop
//                basketDropCurrent2 = getRuntime();
//                latchRot = 1.0;
//            }
//            if (getRuntime() > basketDropCurrent2 + 0.75) { // After .75 sec - close latch servo
//                basketDropCurrent3 = getRuntime();
//                basketDropCurrent2 = Double.MAX_VALUE;
//                latchRot = 0.0;
//            }
//            if (getRuntime() > basketDropCurrent3 + 0.1) { // After 0.25 sec - return bucket
//                basketRot = Range.clip(basketRot, 0.0, 1.0) - 0.03;
//            }
//            if (getRuntime() > basketDropCurrent3 + 0.1 && basketRot <= 0.0) { // After 2 sec - Max the current after the bucket returns
//                basketDropCurrent3 = Double.MAX_VALUE;
//            }
//        }
    }

    public void clawSystem() {
        // --------------------------------------- CLAW SERVO SYSTEM ---------------------------------------
        // (0.2 is closed, 1.0 is open)
        if (gamepad1.b){
            clawRot = 0.2;
        }
        if (gamepad1.a){
            clawRot = 0.8;
        }
    }

    public void verticalSlideSystem() {
        // --------------------------------------- VERTICAL LINEAR SLIDES ---------------------------------------
        // Stop overextension and over retraction of vert linear motor
        if (clickedA) return; // PREVENTS SLIDES MOVING WHILE IN INTAKE OUTTAKE SYSTEM
        liftPosVert = -(vertLinearMotor.getCurrentPosition() - liftPosAdjVert);

        if (vertSlideSensor.isPressed() || gamepad2.dpad_right) { // if slide sensor touched or manual adjustment button pressed
            liftPosAdjVert = vertLinearMotor.getCurrentPosition();
        }

        if (gamepad2.left_stick_y > 0 && !vertSlideSensor.isPressed()) {
            vertLinearPower = gamepad2.left_stick_y;
        } else if (gamepad2.left_stick_y < 0 && liftPosVert < 9800) {
            vertLinearPower = gamepad2.left_stick_y;
        } else { vertLinearPower = 0.0;}
    }

    public void horizonalSlideSystem() {
        // --------------------------------------- HORIZONTAL LINEAR SLIDES ---------------------------------------
        if (clickedA) return; // PREVENTS SLIDES MOVING WHILE IN INTAKE OUTTAKE SYSTEM

        // Stop overextension and over retraction of horizontal linear motor
        liftPosHoriz = -(horizLinearMotor.getCurrentPosition() - liftPosAdjHoriz);
        // if slide sensor touched or manual adjustment button pressed
        if (horizSlideSensor.isPressed() || gamepad2.dpad_left) {
            liftPosAdjHoriz = horizLinearMotor.getCurrentPosition();
        }

        // TODO Set up limits for horizontal linear motor with sensor
        if (gamepad2.right_stick_y > 0) {
            horizLinearPower = -gamepad2.right_stick_y * 0.5;
        } else if (gamepad2.right_stick_y < 0.0 && liftPosHoriz < 8000) {
            horizLinearPower = -gamepad2.right_stick_y * 0.5;
        } else { horizLinearPower = 0.0;}
    }

    public void leadScrewSystem() {
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
    }

    public void updateAttachments() {
        // -------------------------- SET MOTOR POWERS AND SERVO POSITIONS --------------------------
        outtakeMotor.setPower(outtakePower);
        vertLinearMotor.setPower(vertLinearPower);
        horizLinearMotor.setPower(horizLinearPower);
        hangerMotor.setPower(hangerPower);

        clawServo.setPosition(Range.clip(clawRot, 0.2, 1.0));
        armServo.setPosition(Range.clip(armRot, 0.0, 1.0));
        basketServo.setPosition(Range.clip(basketRot, 0.0, 1.0));
        latchServo.setPosition(Range.clip(latchRot,0.0, 1.0));
    }

    @SuppressLint("DefaultLocale")
    public void addTelemetryToDriverStation() {
        telemetry.addData("Runtime", getRuntime());
        telemetry.addData("Clicked A", clickedA);
        telemetry.addData("g2LStickY, g2RStickY", gamepad2.left_stick_y + ", " + gamepad2.right_stick_y );
        telemetry.addData("Vertical, True", liftPosVert + ", " + vertLinearMotor.getCurrentPosition());
        telemetry.addData("Horizontal, True", liftPosHoriz + ", " + horizLinearMotor.getCurrentPosition());
        telemetry.addData("Gyro: ", "Yaw: " + String.format("%.2f", orientation.getYaw(AngleUnit.DEGREES))
                                                + "Roll: " + String.format("%.2f", orientation.getRoll(AngleUnit.DEGREES))
                                                + "Pitch: " + String.format("%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.addData("g1LStickX", gamepad1.left_stick_x);
        telemetry.addData("Slowmode: ", finalSlowMode);
        telemetry.addData("Arm Pos: ", armServo.getPosition());
        telemetry.addData("Basket Pos: ", basketServo.getPosition());
        telemetry.addData("Latch Pos: ", latchServo.getPosition());
        telemetry.addData("Claw Pos: ", clawServo.getPosition());
        telemetry.update();
    }
}
