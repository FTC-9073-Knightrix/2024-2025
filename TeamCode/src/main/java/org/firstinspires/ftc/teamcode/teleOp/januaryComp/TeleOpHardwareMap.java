package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class TeleOpHardwareMap extends OpMode {
    // ------------------------------------ HARDWARE MAP ------------------------------------
    //Drivetrain hardware
    MecanumDrive mecanumDrive;
    public Motor leftFront;
    public Motor rightFront;
    public Motor leftBack;
    public Motor rightBack;

    // Electronics
    public DcMotorEx vertLinearMotorL;
    public DcMotorEx vertLinearMotorR;
    public DcMotor horizLinearMotor;
    public DcMotor hangerMotor;

    public Servo intakeClawServo;
    public Servo intakeTwistServo;
    public Servo intakeArmServo;

    public Servo outtakeClawServo;
    public Servo outtakeArmServo;

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
        // --------------------------------------- INITIALIZATION ---------------------------------------
        telemetry.addData("Initialization","Starting...");

        vertLinearMotorL = hardwareMap.get(DcMotorEx.class, "vertLinearMotorL");
        vertLinearMotorR = hardwareMap.get(DcMotorEx.class, "vertLinearMotorR");
        horizLinearMotor = hardwareMap.get(DcMotor.class, "horizLinearMotor");

        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        intakeTwistServo = hardwareMap.get(Servo.class, "intakeTwistServo");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");

        outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        outtakeArmServo = hardwareMap.get(Servo.class, "outtakeArmServo");

        vertSlideSensor = hardwareMap.get(TouchSensor.class, "vertSlideSensor");
        horizSlideSensor = hardwareMap.get(TouchSensor.class, "horizSlideSensor");
        hangerSensor1 = hardwareMap.get(TouchSensor.class, "hangerSensor1");
        hangerSensor2 = hardwareMap.get(TouchSensor.class, "hangerSensor2");

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

        vertLinearMotorL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        vertLinearMotorR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        horizLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Initialization","Done!");
    }
}
