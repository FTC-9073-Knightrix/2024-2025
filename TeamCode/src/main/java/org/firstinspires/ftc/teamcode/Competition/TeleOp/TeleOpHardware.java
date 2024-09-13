package org.firstinspires.ftc.teamcode.Competition.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class TeleOpHardware extends OpMode {
    //Create the four drive motors and the chassis
    MecanumDrive mecanumDrive;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;

    //Create servos
    Servo launchServo;
    Servo clawServo;
    Servo clawRotateServo;
    Servo outtakeServo;

    //Create the magnet sensors
    TouchSensor touch1;
    TouchSensor touch2;
    TouchSensor slideLimit;

    //Create distance sensors
    DistanceSensor left;
    DistanceSensor right;
    DistanceSensor back;

    //Create the other motors
    DcMotor hanger;
    DcMotor slideMotor;

    //Create the variable that will keep track of the left joystick's x value
    public float leftstick_x = 0;
    public float g1_leftstick_x = 0;
    public float g2_leftstick_x = 0;

    //Create the variable that will keep track of the left joystick's y value
    public float leftstick_y = 0;
    public float g1_leftstick_y = 0;
    public float g2_leftstick_y = 0;

    //Create the variable that will keep track of the right joystick's x value
    public float rightstick_x = 0;
    public float g1_rightstick_x = 0;
    public float g2_rightstick_x = 0;

    //Create the variable that will keep track of the right joystick's y value
    public float rightstick_y = 0;
    public float g1_rightstick_y = 0;
    public float g2_rightstick_y = 0;

    //Create the variable that tracks GamePad1 buttons
    public boolean g1_dpad_down = false;
    public boolean g1_dpad_up = false;
    public boolean g1_dpad_right = false;
    public boolean g1_dpad_left = false;

    //Create the variable that tracks GamePad2 buttons
    public boolean g2_dpad_down = false;
    public boolean g2_dpad_up = false;
    public boolean g2_dpad_right = false;
    public boolean g2_dpad_left = false;

    // GamePad bumpers
    public boolean g1_left_bumper = false;
    public boolean g1_right_bumper = false;
    public boolean g2_right_bumper = false;
    public boolean g2_left_bumper = false;

    //GamePad triggers
    public float g1_right_trigger = 0;
    public float g1_left_trigger = 0;
    public float g2_right_trigger = 0;
    public float g2_left_trigger = 0;

    //GamePad buttons
    public boolean g1_a = false;
    public boolean g1_b = false;
    public boolean g1_x = false;
    public boolean g1_y = false;
    public boolean g2_a = false;
    public boolean g2_b = false;
    public boolean g2_x = false;
    public boolean g2_y = false;

    //Create the gyroscope
    public IMU imu;

    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;

    //Create the variable for the degrees of the robot
    public int gyroDegrees = 0;
    //Create the variable that will calculate the angle of the joystick
    public int myangle = 0;
    //Create the variable that will calculate the power of the robot
    public float mypower = 0;
    //Create the variable that will keep the previous angle of the robot
    public int gyroResetValue = 0;
    //Create the variable that will calculate the rotation of the robot
    public float myrot = 0;

    public void init() {

        //Add the motors to the configuration on the phones
        leftFront = new Motor(hardwareMap, "LF");
        rightFront = new Motor(hardwareMap, "RF");
        leftBack = new Motor(hardwareMap, "LB");
        rightBack = new Motor(hardwareMap, "RB");

        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        //Set up the other motors
        hanger = hardwareMap.dcMotor.get("hanger");
        slideMotor = hardwareMap.dcMotor.get("slideMotor");
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set up the servos
        launchServo = hardwareMap.servo.get("launchServo");
        clawServo = hardwareMap.servo.get("clawServo");
        clawRotateServo = hardwareMap.servo.get("clawRotateServo");
        outtakeServo = hardwareMap.servo.get("outtakeServo");

        //Set up the magnet sensors
        touch1 = hardwareMap.touchSensor.get("hangerLimitBottom");
        touch2 = hardwareMap.touchSensor.get("hangerLimitTop");
        slideLimit = hardwareMap.touchSensor.get("slideLimit");

        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        launchServo.setPosition(-1);
    }
}