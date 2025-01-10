package org.firstinspires.ftc.teamcode.autonomous.competition.january;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.PinpointDrive;

import java.util.Arrays;

@Autonomous(name = "NewSpecimenAuto", group = "Autonomous")
public class FiveSpecimenAuto extends FiveSpecimenActions {
    final double forwardAngle = Math.toRadians(90);
    final double backwardAngle = Math.toRadians(270);
    final double rightAngle = Math.toRadians(0);
    final double leftAngle = Math.toRadians(180);
    @Override
    public void runOpMode() throws InterruptedException {


        Pose2d beginPose = new Pose2d(4, 62, rightAngle);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        VertLinearMotor vertLinearMotor = new VertLinearMotor(hardwareMap);
        OuttakeClawServo outtakeClawServo = new OuttakeClawServo(hardwareMap);
        OuttakeArmServo outtakeArmServo = new OuttakeArmServo(hardwareMap);
        OuttakeTwistServo outtakeTwistServo = new OuttakeTwistServo(hardwareMap);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        // Robot drives to the chamber to hook 1st spec
//        TrajectoryActionBuilder ToBarTraj1 = drive.actionBuilder(beginPose)
//            .strafeToConstantHeading(new Vector2d(4, -30));
        Action ToBarAction1 = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(4, 30), forwardAngle)
//                .strafeToConstantHeading(new Vector2d(20, 0))
                .build();
        // Robot drives to push the 3 samples into the observation zone
        TrajectoryActionBuilder ToObservationTraj = drive.actionBuilder(new Pose2d(4, -30, forwardAngle))
                // first u turn
                .strafeToConstantHeading(new Vector2d(28, -36), baseVelConstraint)
                .splineToConstantHeading(new Vector2d(29,  -36), rightAngle, baseVelConstraint)
                .splineToConstantHeading(new Vector2d(36, -32), forwardAngle, baseVelConstraint)
                .splineToConstantHeading(new Vector2d(36, -16), forwardAngle, baseVelConstraint)
                .splineToConstantHeading(new Vector2d(46, -16), backwardAngle, baseVelConstraint);
//                // down and u turn
//                .splineToConstantHeading(new Vector2d(46, -53), backwardAngle)
//                .splineToConstantHeading(new Vector2d(41, -53), forwardAngle)
//                // go back up and u turn
//                .splineToConstantHeading(new Vector2d(39, -16), forwardAngle)
//                .splineToConstantHeading(new Vector2d(56, -16), backwardAngle)
//                // down and u turn
//                .splineToConstantHeading(new Vector2d(56, -50), backwardAngle)
//                .splineToConstantHeading(new Vector2d(49, -50), forwardAngle)
//                // go back up and u turn
//                .splineToConstantHeading(new Vector2d(49, -16), forwardAngle)
//                .splineToConstantHeading(new Vector2d(59, -16), backwardAngle)
//                // down
//                .splineToConstantHeading(new Vector2d(59, -53), backwardAngle)
//
//                // pickup 1
//                .strafeToConstantHeading(new Vector2d(40, -58), baseVelConstraint)


        if (opModeInInit()) {
//            drive.pinpoint.resetP
//            ition(beginPose);
        }

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Heading during Init", drive.pinpoint.getHeading());
            telemetry.addData("pos", drive.pinpoint.getPositionRR());
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) {
                telemetry.addData("Auton Stopped", "Stop Requested");
                telemetry.update();
                return;
            }
            telemetry.addData("Auton Elapsed Time", getRuntime());
            telemetry.update();

            // -------------------------------------- HOOK 1ST SPEC ------------------------------------
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction1

//                                    vertLinearMotor.liftUpToChamber(),
//                                    outtakeArmServo.clawArmForward()
                            )
//                            vertLinearMotor.hookOnBar(),
//                            outtakeClawServo.openClaw()
                    )
            );
//
//            // -------------------------------------- PUSH BLOCKS ------------------------------------
////            Action PushBlocksAction = PushBlocksTraj.build();
//            Actions.runBlocking(
//                    new ParallelAction(
//                            PushBlocksAction,
//                            outtakeArmServo.clawArmBack(),
//                            outtakeTwistServo.twistToBearingsUp()
//                    )
//            );
////
//            // -------------------------------------- PICKUP 2ND SPEC ------------------------------------
//            Action ToObsvAction1 = ToObsvTraj1.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToObsvAction1,
//                                    clawArm.clawArmBack()
//                            ),
//                            new SleepAction(
//                                    0.5
//                            ),
//                            claw.closeClaw(),
//                            vertLinearMotor.liftOffWall()
//                    )
//            );
//
//            // -------------------------------------- HOOK 2ND SPEC ------------------------------------
//            Action ToBarAction2 = ToBarTraj2.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToBarAction2,
//                                    vertLinearMotor.liftUpToChamber(),
//                                    clawArm.clawArmForward()
//                            ),
//                            vertLinearMotor.hookOnBar(),
//                            claw.openClaw()
//                    )
//            );
//
//            // -------------------------------------- PICKUP 3RD SPEC ------------------------------------
//            Action ToObsvAction2 = ToObsvTraj2.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToObsvAction2,
//                                    vertLinearMotor.liftDown(),
//                                    clawArm.clawArmBack()
//                            ),
//                            new SleepAction(
//                                    0.5
//                            ),
//                            claw.closeClaw(),
//                            vertLinearMotor.liftOffWall()
//                    )
//            );
//
//            // -------------------------------------- HOOK 3RD SPEC ------------------------------------
//            Action ToBarAction3 = ToBarTraj3.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToBarAction3,
//                                    vertLinearMotor.liftUpToChamber(),
//                                    clawArm.clawArmForward()
//                            ),
//                            vertLinearMotor.hookOnBar(),
//                            claw.openClaw()
//                    )
//            );
//
//            // -------------------------------------- PICKUP 4TH SPEC ------------------------------------
//            Action ToObsvAction3 = ToObsvTraj3.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToObsvAction3,
//                                    vertLinearMotor.liftDown(),
//                                    clawArm.clawArmBack()
//                            ),
//                            new SleepAction(
//                                    0.5
//                            ),
//                            claw.closeClaw(),
//                            vertLinearMotor.liftOffWall()
//                    )
//            );
//
//            // -------------------------------------- HOOK 4TH SPEC ------------------------------------
//            Action ToBarAction4 = ToBarTraj4.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToBarAction4,
//                                    vertLinearMotor.liftUpToChamber(),
//                                    clawArm.clawArmForward()
//                            ),
//                            vertLinearMotor.hookOnBar(),
//                            claw.openClaw()
//                    )
//            );
//
//            // -------------------------------------- PICKUP 5TH SPEC ------------------------------------
//            Action ToObsvAction4 = ToObsvTraj4.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToObsvAction3,
//                                    vertLinearMotor.liftDown(),
//                                    clawArm.clawArmBack()
//                            ),
//                            new SleepAction(
//                                    0.5
//                            ),
//                            claw.closeClaw(),
//                            vertLinearMotor.liftOffWall()
//                    )
//            );
//
//            // -------------------------------------- HOOK 5TH SPEC ------------------------------------
//            Action ToBarAction5 = ToBarTraj5.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ToBarAction5,
//                                    vertLinearMotor.liftUpToChamber(),
//                                    clawArm.clawArmForward()
//                            ),
//                            vertLinearMotor.hookOnBar(),
//                            claw.openClaw()
//                    )
//            );
//
//            // -------------------------------------- PARK ------------------------------------
//            Action ParkAction = ParkTraj.build();
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    ParkAction,
//                                    vertLinearMotor.liftDown()
//                            )
//                    )
//            );
        }
    }
}
