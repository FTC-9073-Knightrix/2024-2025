package org.firstinspires.ftc.teamcode.autonomous.competition.january;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinMax;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.onbotjava.handlers.file.NewFile;
import org.firstinspires.ftc.teamcode.autonomous.PinpointDrive;

import java.util.Arrays;

@Autonomous(name = "Five Specimen Auto Comp Walls")
public class FiveSpecimenAutoCompWalls extends AutoActions {
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
        VelConstraint maxSpeedConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(65.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        VelConstraint IntoZoneVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        AccelConstraint backToZoneAccelConstraint = new ProfileAccelConstraint(-30, 70);
        AccelConstraint maxAccelConstraint = new ProfileAccelConstraint(-40.0, 70);

        Action PreloadHookLine = drive.actionBuilder(new Pose2d(0, 0, 0))
                // To Bar 1
                .splineToConstantHeading(new Vector2d(32, 0), 0, maxSpeedConstraint, maxAccelConstraint)
                .build();

        Action PushBlocks = drive.actionBuilder(new Pose2d(32, 0, 0))

                // Curve out
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(32, -32), Math.toRadians(0), baseVelConstraint)
                // Straight forward
                .strafeToConstantHeading(new Vector2d(42, -32))

                // ( curve
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -36), Math.toRadians(270), baseVelConstraint)
                // ) curve
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -42), Math.toRadians(180), baseVelConstraint)
                // Straight back
                .strafeToConstantHeading(new Vector2d(8, -42))


                // Straight forward 2
                .strafeToConstantHeading(new Vector2d(42, -42))

                // ( curve 2
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -47), Math.toRadians(270), baseVelConstraint)
                // ) curve 2
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -52), Math.toRadians(180), baseVelConstraint)

                // Straight back 2
                .strafeToConstantHeading(new Vector2d(8, -52))
                // Straight forward 3
                .strafeToConstantHeading(new Vector2d(42, -52))
                // ( curve 3
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -54.5), Math.toRadians(270), baseVelConstraint)
                // ) curve 3
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -57), Math.toRadians(180), baseVelConstraint) // Only changed this line for comp walls

                // Straight back 3
                .strafeToConstantHeading(new Vector2d(15, -57), baseVelConstraint) // Only changed this line for comp walls

                // Curve into the zone
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(2, -36), Math.toRadians(120), baseVelConstraint)
                .build();

        Action ToBar1 = drive.actionBuilder(new Pose2d(2, -36, 0))
                .strafeToConstantHeading(new Vector2d(24, -4), maxSpeedConstraint, maxAccelConstraint)
                .splineToConstantHeading(new Vector2d(32, 2.5), Math.toRadians(0), maxSpeedConstraint, maxAccelConstraint)
                .build();

        Action ToZone1 = drive.actionBuilder(new Pose2d(32, 2.5, 0))
                .strafeToConstantHeading(new Vector2d(2, -36), maxSpeedConstraint, backToZoneAccelConstraint)
                .build();


        Action ToBar2 = drive.actionBuilder(new Pose2d(2, -36, 0))
                .strafeToConstantHeading(new Vector2d(32, 4.5), maxSpeedConstraint, maxAccelConstraint)
                .build();

        Action ToZone2 = drive.actionBuilder(new Pose2d(32, 4.5, 0))
                .strafeToConstantHeading(new Vector2d(2, -36), maxSpeedConstraint, backToZoneAccelConstraint)
                .build();

        Action ToBar3 = drive.actionBuilder(new Pose2d(2, -36, 0))
                .strafeToConstantHeading(new Vector2d(32, 6.6), maxSpeedConstraint, maxAccelConstraint)
                .build();

        Action ToZone3 = drive.actionBuilder(new Pose2d(32, 6.5, 0))
                .strafeToConstantHeading(new Vector2d(2, -36), maxSpeedConstraint, backToZoneAccelConstraint)
                .build();

        Action ToBar4 = drive.actionBuilder(new Pose2d(2, -36, 0))
                .strafeToConstantHeading(new Vector2d(32, 8.5), maxSpeedConstraint, maxAccelConstraint)
                .build();

        Action Park = drive.actionBuilder(new Pose2d(32, 8.5, 0))
//                .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(6, -50), Math.toRadians(270))
                .strafeToConstantHeading(new Vector2d(6, -40), maxSpeedConstraint)
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
                            // ------- 1ST HOOK ------- //
                            new ParallelAction(
                                    PreloadHookLine,
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward()
                            ),
                            vertLinearMotor.hookOnBar(),
                            outtakeClawServo.openClaw(),

                            // ------ PUSH THE BLOCKS AND GRAB 2ND SPECIMEN ------ //
                            new ParallelAction(
                                    outtakeArmServo.clawArmBack(),
                                    outtakeTwistServo.twistToBearingsUp(),
                                    PushBlocks
                            ),
                            outtakeClawServo.closeClaw(),

                            // ------- TO BAR 2ND HOOK------- //
                            new ParallelAction(
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward(),
                                    outtakeTwistServo.twistToBearingsDown(),
                                    ToBar1
                            ),
                            vertLinearMotor.hookOnBar(),
                            new ParallelAction(
                                    ToZone1,
                                    outtakeClawServo.openClaw(),
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
                                    ToZone3,
                                    outtakeArmServo.clawArmBack(),
                                    outtakeTwistServo.twistToBearingsUp()
                            ),
                            outtakeClawServo.closeClaw(),
                            new ParallelAction(
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward(),
                                    outtakeTwistServo.twistToBearingsDown(),
                                    ToBar4
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
