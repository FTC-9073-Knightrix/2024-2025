package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto Test", group = "Autonomous")
public class AutoMethodsTest extends AutoMethods {
    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        Pose2d beginPose = new Pose2d(8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8, -62, Math.toRadians(270)));
        drive.updatePoseEstimate();

        int specimenAboveChamberHeight = -2287;
        int specimenHookedOntoChamberHeight = -1600;

        if (opModeInInit()) {
            clawServo.setPosition(0.5);
            latchServo.setPosition(1.0);
            armServo.setPosition(0.05);
        }

        // In between init and start
        while (!isStopRequested() && !opModeIsActive()) {

        }
        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) return;
            runtime.reset();
            armServo.setPosition(0.55);
            latchServo.setPosition(0.0);

            Actions.runBlocking( // go a bit in front of bar
                    drive.actionBuilder(beginPose)
                            .lineToY(-34)
                            .build()
            );
            vertLinearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            vertLinearMotor.setTargetPosition(specimenAboveChamberHeight);
            vertLinearMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            vertLinearMotor.setPower(-1);
            while (vertLinearMotor.isBusy() && opModeIsActive()) { // raise slide
                vertLinearMotor.setPower(-1);
            }
            vertLinearMotor.setPower(0);

            drive.updatePoseEstimate();
            Actions.runBlocking( // drive fully into bar
                    drive.actionBuilder(new Pose2d(8, -34,Math.toRadians(270)))
                            .lineToY(-32)
                            .build()
            );


            vertLinearMotor.setTargetPosition(specimenHookedOntoChamberHeight);
            runtime.reset();
            vertLinearMotor.setPower(0.4);
//            double current1 = getRuntime();
            while (vertLinearMotor.isBusy() && opModeIsActive()) { // hook onto bar
                vertLinearMotor.setPower(0.4);
            }
            clawServo.setPosition(0.2);
            vertLinearMotor.setPower(0);
            vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (!vertSlideSensor.isPressed() && opModeIsActive()) { // bring slide down
                vertLinearMotor.setPower(1);
            }
            vertLinearMotor.setPower(0);
            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Go to obv zone
            Actions.runBlocking(drive.actionBuilder(new Pose2d(8, -32, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(8, -34), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(46, -58, Math.toRadians(92.5)), Math.toRadians(270))
                .waitSeconds(1)
                .lineToY(-66)
                .build()
            );

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

            Actions.runBlocking(drive.actionBuilder(new Pose2d(46, -62, Math.toRadians(90)))
                .splineToConstantHeading(new Vector2d(46, -61), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-4, -33, Math.toRadians(270)), Math.toRadians(90))
                .build()
            );

            vertLinearMotor.setTargetPosition(specimenAboveChamberHeight);
            runtime.reset();
            vertLinearMotor.setPower(-1);
            while (vertLinearMotor.isBusy() && opModeIsActive()) { // raise slide
                vertLinearMotor.setPower(-1);
            }
            vertLinearMotor.setPower(0);

            drive.updatePoseEstimate();

            Actions.runBlocking( // drive fully into bar
                    drive.actionBuilder(new Pose2d(4, -34,Math.toRadians(270)))
                            .lineToY(-31.5)
                            .build()
            );

            vertLinearMotor.setTargetPosition(specimenHookedOntoChamberHeight);
            runtime.reset();
            vertLinearMotor.setPower(0.4);
//            double current1 = getRuntime();
            while (vertLinearMotor.isBusy() && opModeIsActive()) { // hook onto bar
                vertLinearMotor.setPower(0.4);
            }
            clawServo.setPosition(0.2);
            vertLinearMotor.setPower(0);
            vertLinearMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (!vertSlideSensor.isPressed() && opModeIsActive()) { // bring slide down
                vertLinearMotor.setPower(1);
            }
            vertLinearMotor.setPower(0);
            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Actions.runBlocking(drive.actionBuilder(new Pose2d(4, -32, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(4, -34), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d( 35, -34), Math.toRadians(90))

                .splineToConstantHeading(new Vector2d(35, -15), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(44, -15), Math.toRadians(270))
                .lineToY(-55)

                .lineToY(-15)
                .splineToConstantHeading(new Vector2d(56, -15), Math.toRadians(270))
                .lineToY(-55)

                .lineToY(-15)
                .splineToConstantHeading(new Vector2d(61, -15), Math.toRadians(270))
                .lineToY(-55)
                .build()
            );
        }
    }
}