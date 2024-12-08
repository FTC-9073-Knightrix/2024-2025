package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "SpecimenAuto", group = "Autonomous")
public class SpecimenAuto extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        Pose2d beginPose = new Pose2d(8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8, -62, Math.toRadians(270)));
        drive.updatePoseEstimate();

        // Drives a bit in front of bar #1
        Action action1 = drive.actionBuilder(beginPose)
                .lineToY(-34)
                .build();

        // Drives into bar #1
        Action action2 = drive.actionBuilder(new Pose2d(8, -34, Math.toRadians(270)))
                .lineToY(-32)
                .build();

        // Drives to observation zone
        Action action3 = drive.actionBuilder(new Pose2d(8, -32, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(8, -34), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46, -58, Math.toRadians(89)), Math.toRadians(270))
                .waitSeconds(0.75)
                .lineToY(-66)
                .build();

        // Drives back to bar a bit in front
        Action action4 = drive.actionBuilder(new Pose2d(46, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(46, -61), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-4, -34, Math.toRadians(270)), Math.toRadians(90))
                .build();

        // Drives into bar #2
        Action action5 = drive.actionBuilder(new Pose2d(-4, -34
                ,Math.toRadians(270)))
                .lineToY(-32)
                .build();

        // Pushes blocks and finishes in observation zone
        Action action6 = drive.actionBuilder(new Pose2d(-4, -32, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(-4, -34), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d( 35, -34), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(35, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(45, -15), Math.toRadians(270))
                .lineToY(-61)


                .lineToY(-15)
                .splineToConstantHeading(new Vector2d(55, -15), Math.toRadians(270))
                .lineToY(-61)
//                .turnTo(Math.toRadians(180))
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
            runtime.reset();
            armServo.setPosition(0.5);
            latchServo.setPosition(0.0);

            Actions.runBlocking(new SequentialAction(action1));// go a bit in front of bar

//                    drive.actionBuilder(beginPose)
//                            .lineToY(-34)
//                            .build()
//            );
//            vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            vertLinearMotor.setTargetPosition(specimenAboveChamberHeight);
//            vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            vertLinearMotor.setPower(-1);
//            while (vertLinearMotor.isBusy() && opModeIsActive()) { // raise slide
//                vertLinearMotor.setPower(-1);
//            }
//            vertLinearMotor.setPower(0);

            raiseSlideAboveChamber();
            Actions.runBlocking((new SequentialAction(action2)));// drive fully into bar
//                    drive.actionBuilder(new Pose2d(8, -34,Math.toRadians(270)))
//                            .lineToY(-32)
//                            .build()
//            );
            clipSpecimenOntoChamberAndDropSlide();
//            vertLinearMotor.setTargetPosition(specimenHookedOntoChamberHeight);
//            runtime.reset();
//            vertLinearMotor.setPower(0.4);
//            while (vertLinearMotor.isBusy() && opModeIsActive()) { // hook onto bar
//                vertLinearMotor.setPower(0.4);
//            }
//            clawServo.setPosition(0.2);
//            sleep(50);
//            vertLinearMotor.setPower(0);
//            vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            while (!vertSlideSensor.isPressed() && opModeIsActive()) { // bring slide down
//                vertLinearMotor.setPower(1);
//            }
//            vertLinearMotor.setPower(0);
//            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Go to obv zone
            Actions.runBlocking(new SequentialAction(action3));
//                .splineToConstantHeading(new Vector2d(8, -34), Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(46, -58, Math.toRadians(92.5)), Math.toRadians(270))
//                .waitSeconds(1)
//                .lineToY(-66)
//                .build()
//            );

            sleep(500);

            clawServo.setPosition(0.55); // grab specimen from human player
            sleep(100);
            vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertLinearMotor.setTargetPosition(-400);
            vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (vertLinearMotor.isBusy() && opModeIsActive()) { // pull up from wall
                vertLinearMotor.setPower(-0.75);
            }
            vertLinearMotor.setPower(0);

            Actions.runBlocking(new SequentialAction(action4));
//                    drive.actionBuilder(new Pose2d(46, -62, Math.toRadians(90)))
//                .splineToConstantHeading(new Vector2d(46, -61), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(-4, -33, Math.toRadians(270)), Math.toRadians(90))
//                .build()
//            );
            raiseSlideAboveChamber();
//            vertLinearMotor.setTargetPosition(specimenAboveChamberHeight);
//            runtime.reset();
//            vertLinearMotor.setPower(-1);
//            while (vertLinearMotor.isBusy() && opModeIsActive()) { // raise slide
//                vertLinearMotor.setPower(-1);
//            }
//            vertLinearMotor.setPower(0);

            Actions.runBlocking( new SequentialAction(action5));// drive fully into bar
//                    drive.actionBuilder(new Pose2d(-4, -34,Math.toRadians(270)))
//                            .lineToY(-31.5)
//                            .build()
//            );

            clipSpecimenOntoChamberAndDropSlide();
//            vertLinearMotor.setTargetPosition(specimenHookedOntoChamberHeight);
//            runtime.reset();
//            vertLinearMotor.setPower(0.4);
//            while (vertLinearMotor.isBusy() && opModeIsActive()) { // hook onto bar
//                vertLinearMotor.setPower(0.4);
//            }
//            clawServo.setPosition(0.2);
//            sleep(50);
//            vertLinearMotor.setPower(0);
//            vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            while (!vertSlideSensor.isPressed() && opModeIsActive()) { // bring slide down
//                vertLinearMotor.setPower(1);
//            }
//            vertLinearMotor.setPower(0);
//            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Actions.runBlocking(new SequentialAction(action6));
//                    drive.actionBuilder(new Pose2d(4, -32, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(4, -34), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d( 35, -34), Math.toRadians(90))
//
//                .splineToConstantHeading(new Vector2d(35, -15), Math.toRadians(90))
//                .splineToConstantHeading(new Vector2d(47, -15), Math.toRadians(270))
//                .lineToY(-60)
//
//
//                .lineToY(-15)
//                .splineToConstantHeading(new Vector2d(55, -15), Math.toRadians(270))
//                .lineToY(-60)
//                .turnTo(Math.toRadians(180))
//                .build()
//            );
        }
    }
}