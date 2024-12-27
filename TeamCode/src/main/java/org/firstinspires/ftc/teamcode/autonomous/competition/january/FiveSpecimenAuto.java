package org.firstinspires.ftc.teamcode.autonomous.competition.january;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.PinpointDrive;

@Autonomous(name = "ParallelActionsTest", group = "Autonomous")
public class FiveSpecimenAuto extends FiveSpecimenActions {
    final double forwardAngle = Math.toRadians(90);
    final double backwardAngle = Math.toRadians(270);
    final double rightAngle = Math.toRadians(0);
    final double leftAngle = Math.toRadians(180);
    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d beginPose = new Pose2d(-8, -62, forwardAngle);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        VertLinearMotor vertLinearMotor = new VertLinearMotor(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        ClawArm clawArm = new ClawArm(hardwareMap);

        // Robot drives to the chamber to hook 1st spec
        TrajectoryActionBuilder ToBarTraj1 = drive.actionBuilder(beginPose)
                .lineToX(-32);
        Action ToBarAction1 = ToBarTraj1.build();


        // Robot drives to push the 3 samples into the observation zone
        TrajectoryActionBuilder PushBlocksTraj = ToBarTraj1.endTrajectory().fresh()
                // first u turn
                .strafeToConstantHeading(new Vector2d(28, -38))
                .splineToConstantHeading(new Vector2d(29, -38), rightAngle)
                .splineToConstantHeading(new Vector2d(36, -32), forwardAngle)
                .splineToConstantHeading(new Vector2d(36, -15), forwardAngle)
                .splineToConstantHeading(new Vector2d(44, -15), backwardAngle);

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
                                    vertLinearMotor.liftUpToChamber()
                            ),
                            vertLinearMotor.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PUSH BLOCKS ------------------------------------
            Action PushBlocksAction = PushBlocksTraj.build();
            Actions.runBlocking(
                    new ParallelAction(
                            PushBlocksAction,
                            vertLinearMotor.liftDown()
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
                            vertLinearMotor.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 2ND SPEC ------------------------------------
            Action ToBarAction2 = ToBarTraj2.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction2,
                                    vertLinearMotor.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            vertLinearMotor.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PICKUP 3RD SPEC ------------------------------------
            Action ToObsvAction2 = ToObsvTraj2.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToObsvAction2,
                                    vertLinearMotor.liftDown(),
                                    clawArm.clawArmBack()
                            ),
                            new SleepAction(
                                    0.5
                            ),
                            claw.closeClaw(),
                            vertLinearMotor.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 3RD SPEC ------------------------------------
            Action ToBarAction3 = ToBarTraj3.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction3,
                                    vertLinearMotor.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            vertLinearMotor.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PICKUP 4TH SPEC ------------------------------------
            Action ToObsvAction3 = ToObsvTraj3.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToObsvAction3,
                                    vertLinearMotor.liftDown(),
                                    clawArm.clawArmBack()
                            ),
                            new SleepAction(
                                    0.5
                            ),
                            claw.closeClaw(),
                            vertLinearMotor.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 4TH SPEC ------------------------------------
            Action ToBarAction4 = ToBarTraj4.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction4,
                                    vertLinearMotor.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            vertLinearMotor.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PICKUP 5TH SPEC ------------------------------------
            Action ToObsvAction4 = ToObsvTraj4.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToObsvAction3,
                                    vertLinearMotor.liftDown(),
                                    clawArm.clawArmBack()
                            ),
                            new SleepAction(
                                    0.5
                            ),
                            claw.closeClaw(),
                            vertLinearMotor.liftOffWall()
                    )
            );

            // -------------------------------------- HOOK 5TH SPEC ------------------------------------
            Action ToBarAction5 = ToBarTraj5.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction5,
                                    vertLinearMotor.liftUpToChamber(),
                                    clawArm.clawArmForward()
                            ),
                            vertLinearMotor.hookOnBar(),
                            claw.openClaw()
                    )
            );

            // -------------------------------------- PARK ------------------------------------
            Action ParkAction = ParkTraj.build();
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ParkAction,
                                    vertLinearMotor.liftDown()
                            )
                    )
            );
        }
    }
}
