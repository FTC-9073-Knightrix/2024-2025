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
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.PinpointDrive;

import java.util.Arrays;

@Autonomous(name = "Four Specimen Auto Optimized")
public class FourSpecimenAutoNew extends AutoActions {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        drive.updatePoseEstimate();

        VertLinearMotor vertLinearMotor = new VertLinearMotor(hardwareMap);
        OuttakeClawServo outtakeClawServo = new OuttakeClawServo(hardwareMap);
        OuttakeArmServo outtakeArmServo = new OuttakeArmServo(hardwareMap);
        OuttakeTwistServo outtakeTwistServo = new OuttakeTwistServo(hardwareMap);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        VelConstraint maxVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(60.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        Action ToBarAction1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                // To Bar 1
                .splineToConstantHeading(new Vector2d(32, 0), 0)
                .build();

        TrajectoryActionBuilder PushBlocksTraj = drive.actionBuilder(new Pose2d(32, 0, 0))
                // Curve out
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(32, -32), Math.toRadians(0), baseVelConstraint)
                // Straight forward
                .strafeToConstantHeading(new Vector2d(42, -32), baseVelConstraint)

                // ( curve
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -36), Math.toRadians(270), baseVelConstraint)
                // ) curve
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -42), Math.toRadians(180), baseVelConstraint)
                // Straight back
                .strafeToConstantHeading(new Vector2d(6, -42))


                // Straight forward 2
                .strafeToConstantHeading(new Vector2d(42, -42))

                // ( curve 2
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -47), Math.toRadians(270), baseVelConstraint)
                // ) curve 2
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -52), Math.toRadians(180), baseVelConstraint)

                // Straight back 2
                .strafeToConstantHeading(new Vector2d(6, -52))

                // To Left
                .strafeToConstantHeading(new Vector2d(6, -36))

                // Back into zone
                .strafeToConstantHeading(new Vector2d(2, -36));

        Action PushBlocks = PushBlocksTraj.build();

        Action ToBar1 = drive.actionBuilder(new Pose2d(2, -36, 0))
                .strafeToConstantHeading(new Vector2d(28, 2))
                .strafeToConstantHeading(new Vector2d(32, 2))
                .build();

        Action ToZone1 = drive.actionBuilder(new Pose2d(32, 2, 0))
                .strafeToConstantHeading(new Vector2d(2, -36))
                .build();

        Action ToBar2 = drive.actionBuilder(new Pose2d(2, -36, 0))
                .strafeToConstantHeading(new Vector2d(28, 4))
                .strafeToConstantHeading(new Vector2d(32, 4))
                .build();

        Action ToZone2 = drive.actionBuilder(new Pose2d(32, 4, 0))
                .strafeToConstantHeading(new Vector2d(2, -36))
                .build();

        Action ToBar3 = drive.actionBuilder(new Pose2d(2, -36, 0))
                .strafeToConstantHeading(new Vector2d(28, 6))
                .strafeToConstantHeading(new Vector2d(32, 6))
                .build();

        Action Park = drive.actionBuilder(new Pose2d(32, 0, 0))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(6, -50), Math.toRadians(270))
                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Heading during Init", drive.pinpoint.getHeading());
            telemetry.addData("pos", drive.pose);
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

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    ToBarAction1,
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward()
                            ),
                            vertLinearMotor.hookOnBar(),
                            outtakeClawServo.openClaw(),
                            new ParallelAction(
                                    outtakeArmServo.clawArmBack(),
                                    outtakeTwistServo.twistToBearingsUp(),
                                    PushBlocks
                            ),
                            outtakeClawServo.closeClaw(),
                            new ParallelAction(
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward(),
                                    outtakeTwistServo.twistToBearingsDown(),
                                    ToBar1
                            ),
                            vertLinearMotor.hookOnBar(),
                            outtakeClawServo.openClaw(),
                            new ParallelAction(
                                    ToZone1,
                                    outtakeArmServo.clawArmBack(),
                                    outtakeTwistServo.twistToBearingsUp()
                            ),
                            outtakeClawServo.closeClaw(),
                            new ParallelAction(
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward(),
                                    outtakeTwistServo.twistToBearingsDown(),
                                    ToBar2
                            ),
                            vertLinearMotor.hookOnBar(),
                            outtakeClawServo.openClaw(),

                            new ParallelAction(
                                    ToZone2,
                                    outtakeArmServo.clawArmBack(),
                                    outtakeTwistServo.twistToBearingsUp()
                            ),
                            outtakeClawServo.closeClaw(),

                            new ParallelAction(
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward(),
                                    outtakeTwistServo.twistToBearingsDown(),
                                    ToBar3
                            ),
                            vertLinearMotor.hookOnBar(),
                            outtakeClawServo.openClaw(),
                            new ParallelAction(
                                    Park,
                                    outtakeArmServo.clawArmBack(),
                                    outtakeTwistServo.twistToBearingsUp()
                            )
                    )
            );
        }
    }
}
