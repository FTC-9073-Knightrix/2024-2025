package org.firstinspires.ftc.teamcode.autonomous.competition.january;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;

@Autonomous(name = "ParallelActionsTest", group = "Autonomous")
public class FiveSpecimenAuto extends FiveSpecimenActions {
    @Override
    public void runOpMode() throws InterruptedException {
        final double forwardAngle = Math.toRadians(90);
        final double backwardAngle = Math.toRadians(270);
        final double rightAngle = Math.toRadians(0);
        final double leftAngle = Math.toRadians(180);

        Pose2d beginPose = new Pose2d(-8, -62, forwardAngle);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        VertLift lift = new VertLift(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ClawArm clawArm = new ClawArm(hardwareMap);

        // Robot drives to the chamber to hook 1st spec
        TrajectoryActionBuilder ToBarTraj1 = drive.actionBuilder(beginPose)
                .lineToX(-32);
        Action ToBarAction1 = ToBarTraj1.build();

        // Robot drives to push the 3 samples into the observation zone
        TrajectoryActionBuilder PushBlocksTraj = ToBarTraj1.endTrajectory().fresh();

        // Robot drives to the observation zone to pick up the 2nd spec
        TrajectoryActionBuilder ToObsvTraj1 = PushBlocksTraj.endTrajectory().fresh();

        // Starts repeating
        // Robot drives to the chamber to hook 2nd Spec
        TrajectoryActionBuilder ToBarTraj2 = ToObsvTraj1.endTrajectory().fresh();

        // Robot drives to the observation zone to pick up the 3rd spec
        TrajectoryActionBuilder ToObsvTraj2 = ToBarTraj2.endTrajectory().fresh();

        // Robot drives to the chamber to hook 3rd Spec
        TrajectoryActionBuilder ToBarTraj3 = ToObsvTraj2.endTrajectory().fresh();

        // Robot drives to the observation zone to pick up the 4th spec
        TrajectoryActionBuilder ToObsvTraj3 = ToBarTraj3.endTrajectory().fresh();

        // Robot drives to the chamber to hook the 4th spec
        TrajectoryActionBuilder ToBarTraj4 = ToObsvTraj3.endTrajectory().fresh();

        // Robot drives to the observation zone to pick up the 5th spec
        TrajectoryActionBuilder ToObsvTraj4 = ToBarTraj4.endTrajectory().fresh();

        // Robot drives to the chamber to hook the 5th spec
        TrajectoryActionBuilder ToBarTraj5 = ToObsvTraj4.endTrajectory().fresh();

        // Robot drives to observation zone to park the robot
        TrajectoryActionBuilder ParkTraj = ToBarTraj5.endTrajectory().fresh();

        if (opModeInInit()) {
            claw.setPos(clawClosePosition);
            clawArm.setPos(clawArmForwardPosition);
        }

        while (!isStopRequested() && !opModeIsActive()) {
            int position = 0;
            telemetry.addData("Position during Init", position);
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
                                    ToBarAction1,
                                    lift.liftUpToChamber()
                            ),
                            lift.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PUSH BLOCKS ------------------------------------
            Action PushBlocksAction = PushBlocksTraj.build();
            Actions.runBlocking(
                    new ParallelAction(
                            PushBlocksAction,
                            lift.liftDown()
                    )
            );

            // -------------------------------------- PICKUP 2ND SPEC ------------------------------------
            Action ToObsvAction1 = ToObsvTraj1.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToObsvAction1,
                                    clawArm.clawArmBack()
                            ),
                            new SleepAction(
                                    0.5
                            ),
                            claw.closeClaw(),
                            lift.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 2ND SPEC ------------------------------------
            Action ToBarAction2 = ToBarTraj2.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction2,
                                    lift.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            lift.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PICKUP 3RD SPEC ------------------------------------
            Action ToObsvAction2 = ToObsvTraj2.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToObsvAction2,
                                    lift.liftDown(),
                                    clawArm.clawArmBack()
                            ),
                            new SleepAction(
                                    0.5
                            ),
                            claw.closeClaw(),
                            lift.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 3RD SPEC ------------------------------------
            Action ToBarAction3 = ToBarTraj3.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction3,
                                    lift.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            lift.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PICKUP 4TH SPEC ------------------------------------
            Action ToObsvAction3 = ToObsvTraj3.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToObsvAction3,
                                    lift.liftDown(),
                                    clawArm.clawArmBack()
                            ),
                            new SleepAction(
                                    0.5
                            ),
                            claw.closeClaw(),
                            lift.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 4TH SPEC ------------------------------------
            Action ToBarAction4 = ToBarTraj4.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction4,
                                    lift.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            lift.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PICKUP 5TH SPEC ------------------------------------
            Action ToObsvAction4 = ToObsvTraj4.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToObsvAction3,
                                    lift.liftDown(),
                                    clawArm.clawArmBack()
                            ),
                            new SleepAction(
                                    0.5
                            ),
                            claw.closeClaw(),
                            lift.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 5TH SPEC ------------------------------------
            Action ToBarAction5 = ToBarTraj5.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction5,
                                    lift.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            lift.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PARK ------------------------------------
            Action ParkAction = ParkTraj.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ParkAction,
                                    lift.liftDown()
                            )
                    )
            );
        }
    }
}
