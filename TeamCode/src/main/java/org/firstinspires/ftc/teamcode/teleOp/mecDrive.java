package org.firstinspires.ftc.teamcode.teleOp;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;
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

    MecanumDrive mecanumDrive;
    Motor leftFront;
    Motor rightFront;
    Motor leftBack;
    Motor rightBack;


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

        mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        rightBack.setInverted(true);
        rightFront.setInverted(true);

        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    @SuppressLint("DefaultLocale")
    public void loop() {
        mecanumDrive();
        telemetry.addData("Gyro: ", "Yaw: " + String.format("%.2f", orientation.getYaw(AngleUnit.DEGREES))
                + "Roll: " + String.format("%.2f", orientation.getRoll(AngleUnit.DEGREES))
                + "Pitch: " + String.format("%.2f", orientation.getPitch(AngleUnit.DEGREES)));
        telemetry.update();
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


        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.y) {
            imu.resetYaw();
        }

        orientation = imu.getRobotYawPitchRollAngles();
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
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
}
