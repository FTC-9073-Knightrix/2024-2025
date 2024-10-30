package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Mecanum Drive")
public class mecDrive extends OpMode {
    //Driving Variables
    public static final double driveSpeed = 0.66;
    public static final double fastSpeed = 1.0;
    public static final double slowSpeed = 0.25;
    public static double finalSlowMode = 0.0;
    public static boolean slowMode = true;
    public static int gyroAdj = 0;

    //Lifting and Servo Variables
    int liftPos = 0;
    int liftPosAdj = 0;
    double liftPower = 0.0;
    double clawPos = 0.0;
    double servoRot = 0.0;
    double outtakeRot = 0.0;
    boolean startHanging = false;

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

    //Create the gyroscope
    public IMU imu;

    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;

    public void init() {
        //Add the motors to the configuration on the phones
        leftFront = new Motor(hardwareMap, "LF");
        rightFront = new Motor(hardwareMap, "RF");
        leftBack = new Motor(hardwareMap, "LB");
        rightBack = new Motor(hardwareMap, "RB");

        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void loop() {
        mecanumDrive();
    }

    public void mecanumDrive(){

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
    }
}
