package org.firstinspires.ftc.teamcode.teleOp.januaryComp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
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
    public DcMotorEx vertLinearMotor;
    public DcMotor horizLinearMotor;
    public DcMotor hangerMotor;

    public Servo intakeClawServo;
    public Servo intakeTwistServo;
    public Servo intakeArmServo;

    public Servo outtakeClawServo;
    public Servo outtakeTwistServo;
    public Servo outtakeArmServo;

    public TouchSensor vertSlideSensor;
    public TouchSensor horizSlideSensor;
    public TouchSensor hangerSensor1;

    // Both the color sensor and intake distance sensor are on the REV Color Sensor V3
    public NormalizedColorSensor colorSensor;
    public DistanceSensor intakeDistanceSensor;

    //Create the gyroscope
    public IMU imu;

    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;


    @Override
    public void init() {
        // --------------------------------------- INITIALIZATION ---------------------------------------
        telemetry.addData("Initialization","Starting...");

        vertLinearMotor = hardwareMap.get(DcMotorEx.class, "vertLinearMotor");
        horizLinearMotor = hardwareMap.get(DcMotor.class, "horizLinearMotor");

        intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
        intakeTwistServo = hardwareMap.get(Servo.class, "intakeTwistServo");
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");

        outtakeClawServo = hardwareMap.get(Servo.class, "outtakeClawServo");
        outtakeArmServo = hardwareMap.get(Servo.class, "outtakeArmServo");
        outtakeTwistServo = hardwareMap.get(Servo.class, "outtakeTwistServo");

        vertSlideSensor = hardwareMap.get(TouchSensor.class, "vertSlideSensor");
        horizSlideSensor = hardwareMap.get(TouchSensor.class, "horizSlideSensor");
        hangerSensor1 = hardwareMap.get(TouchSensor.class, "hangerSensor1");

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "intakeDistanceSensor");

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
}
