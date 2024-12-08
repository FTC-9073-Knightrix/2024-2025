package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="SampleAuto", group="Autonomous")
public class SampleAuto extends AutoMethods{

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        Pose2d beginPose = new Pose2d(-8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8, -62, Math.toRadians(270)));
        drive.updatePoseEstimate();

        double basketRotation = 0.0;
        // Drives into bar #1
        Action action1 = drive.actionBuilder(beginPose)
                .lineToY(-34)
                .build();
        // Drives into bar #1
        Action action2 = drive.actionBuilder(new Pose2d(-8, -34, Math.toRadians(270)))
                .lineToY(-32)
                .build();

        Action action3 = drive.actionBuilder(new Pose2d(-8, -32, Math.toRadians(270)))
                .lineToY(-44)
                .turnTo(Math.toRadians(180))
                .lineToX(-35)
                .strafeToConstantHeading(new Vector2d(-35, -25))
                .strafeToConstantHeading(new Vector2d(-35, -10))
                .build();
        Action action4 = drive.actionBuilder(new Pose2d(35, -10, Math.toRadians(180)))
                .strafeToConstantHeading(new Vector2d(-26, -10))
                .build();

        if (opModeInInit()) {
            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            clawServo.setPosition(0.55);
            latchServo.setPosition(1.0);
            armServo.setPosition(0.05);
        }

        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) return;
            armServo.setPosition(0.5);
            latchServo.setPosition(0.0);
            Actions.runBlocking(new SequentialAction(action1));
            raiseSlideAboveChamber();
            Actions.runBlocking(new SequentialAction(action2));
            clipSpecimenOntoChamberAndDropSlide();
            Actions.runBlocking(new SequentialAction(action3));
//            while (basketServo.getPosition() < 1 && opModeIsActive()) {
//                basketRotation = Range.clip(basketRotation, 0.0, 1.0) + 0.05;
//                basketServo.setPosition(basketRotation);
//            }
            basketServo.setPosition(1.0); // lvl 1 ascend
            Actions.runBlocking(new SequentialAction(action4));
        }

    }
}
