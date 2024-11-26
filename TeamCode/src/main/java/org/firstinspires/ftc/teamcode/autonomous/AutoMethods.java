package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class AutoMethods extends LinearOpMode {
    // Creating Servos
    CRServo outtakeServo;
    Servo armServo;
    Servo clawServo;
    Servo basketServo;

    // Touch sensors
    TouchSensor vertSlideLimit;
    TouchSensor horizSlideLimit;
    DcMotor vertLinearMotor;
    DcMotor horizLinearMotor;

    MecanumDrive mecanumDrive;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    public IMU imu;
    public ElapsedTime runtime;

    public void initRobot() {
        telemetry.addData("Initialization", "Starting...");
        telemetry.addData("Initialization", "Done!");

        vertLinearMotor = hardwareMap.dcMotor.get("vertLinearMotor");
        horizLinearMotor = hardwareMap.dcMotor.get("horizLinearMotor");

        armServo = hardwareMap.servo.get("rotateServo");
        clawServo = hardwareMap.servo.get("clawServo");
        basketServo = hardwareMap.servo.get("basketServo");
        outtakeServo = hardwareMap.crservo.get("outtakeServo");

        vertSlideLimit = hardwareMap.touchSensor.get("vertSlideLimit");
        horizSlideLimit = hardwareMap.touchSensor.get("horizSlideLimit");

        vertLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = new Motor(hardwareMap, "LF");
        rightFront = new Motor(hardwareMap, "RF");
        leftBack = new Motor(hardwareMap, "LB");
        rightBack = new Motor(hardwareMap, "RB");

        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }



    public void VertliftArm(int liftHeight, double liftSpeed, double timeInSeconds) {
        vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertLinearMotor.setTargetPosition(liftHeight);
        vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        vertLinearMotor.setPower(liftSpeed);
        while(vertLinearMotor.isBusy() && opModeIsActive()) {
            vertLinearMotor.setPower(liftSpeed);
        }
        vertLinearMotor.setPower(0);
        sleep(500);
    }

    public void HorizliftArm(int liftHeight, double liftSpeed, double timeInSeconds) {
        horizLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        horizLinearMotor.setTargetPosition(liftHeight);
        horizLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        horizLinearMotor.setPower(liftSpeed);
        while (horizLinearMotor.isBusy() && opModeIsActive()) {
            horizLinearMotor.setPower(liftSpeed);
        }
        horizLinearMotor.setPower(0);
        sleep(500);
    }


    public void operateOuttakeServo(double speed, double duration) {
        outtakeServo.setPower(speed);
        sleep((long) (duration * 1000));
        outtakeServo.setPower(0);
    }
}