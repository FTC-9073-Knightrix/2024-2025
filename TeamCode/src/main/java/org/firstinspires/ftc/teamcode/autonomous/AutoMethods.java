package org.firstinspires.ftc.teamcode.autonomous;
import android.icu.lang.UCharacter;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
//import com.arcrobotics.ftclib.drivebase.MecanumDrive;
//import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public abstract class AutoMethods extends LinearOpMode {
    // Electronics
    public DcMotorEx intakeMotor;
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
    public ElapsedTime runtime;

    //Create the orientation variable for the robot position
    public YawPitchRollAngles orientation;

    int specimenAboveChamberHeight = -2190;
    int specimenHookedOntoChamberHeight = -1600;
    public void initRobot() {
        runtime = new ElapsedTime();
        // --------------------------------------- INITIALIZATION ---------------------------------------
        telemetry.addData("Initialization","Starting...");

        intakeMotor= hardwareMap.get(DcMotorEx.class, "intakeMotor"); // Expansion Hub Motor 0
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

        //Add the gyroscope to the configuration on the phones
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu.initialize(new IMU.Parameters(orientationOnRobot));

        vertLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizLinearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Initialization","Done!");
    }
    public void raiseSlideAboveChamber() {
        vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertLinearMotor.setTargetPosition(specimenAboveChamberHeight);
        vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        vertLinearMotor.setPower(-1);
        while (vertLinearMotor.isBusy() && opModeIsActive()) { // raise slide
            vertLinearMotor.setPower(-1);
        }
        vertLinearMotor.setPower(0);
    }
    public void clipSpecimenOntoChamberAndDropSlide() {
        vertLinearMotor.setTargetPosition(specimenHookedOntoChamberHeight);
        runtime.reset();
        vertLinearMotor.setPower(0.4);
        while (vertLinearMotor.isBusy() && opModeIsActive()) { // hook onto bar
            vertLinearMotor.setPower(0.4);
        }
        clawServo.setPosition(0.2);
        sleep(50);
        vertLinearMotor.setPower(0);
        vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (!vertSlideSensor.isPressed() && opModeIsActive()) { // bring slide down
            vertLinearMotor.setPower(1);
        }
        vertLinearMotor.setPower(0);
        vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}