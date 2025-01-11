package org.firstinspires.ftc.teamcode.autonomous.competition.january;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.PinpointDrive;

import java.util.Arrays;

@Autonomous(name = "Four Specimen Auto")
public class FourSpecimenAuto extends AutoActions {
//    final double forwardAngle = Math.toRadians(90);
//    final double backwardAngle = Math.toRadians(270);
//    final double rightAngle = Math.toRadians(0);
//    final double leftAngle = Math.toRadians(180);

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
                new TranslationalVelConstraint(20.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        Action ToBarAction1 = drive.actionBuilder(new Pose2d(0, 0, 0))
                // To Bar 1
                .splineToConstantHeading(new Vector2d(32, 0), 0)
                .build();

        Action Strafe1 = drive.actionBuilder(new Pose2d(32, 0, 0))
                // Curve out
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(22, -32), Math.toRadians(270))
                // Straight forward
                .strafeToConstantHeading(new Vector2d(42, -32))

                // ( curve
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -37), Math.toRadians(270))
                // ) curve
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -43), Math.toRadians(180))
                // Straight back
                .strafeToConstantHeading(new Vector2d(6, -43))


                // Straight forward 2
                .strafeToConstantHeading(new Vector2d(42, -43))

                // ( curve 2
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -48.5), Math.toRadians(270))
                // ) curve 2
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -55), Math.toRadians(180))

                // Straight back 2
                .strafeToConstantHeading(new Vector2d(6, -55))

                // To Left
                .strafeToConstantHeading(new Vector2d(6, -36))

                // Back into zone
                .strafeToConstantHeading(new Vector2d(1, -36))
                .build();

        Action ToBar1 = drive.actionBuilder(new Pose2d(1, -36, 0))
                .strafeToConstantHeading(new Vector2d(25, 2))
                .strafeToConstantHeading(new Vector2d(32, 2))
                .build();

        Action ToZone1 = drive.actionBuilder(new Pose2d(32, 2, 0))
                .strafeToConstantHeading(new Vector2d(1, -36))
                .build();

        Action ToBar2 = drive.actionBuilder(new Pose2d(1, -36, 0))
                .strafeToConstantHeading(new Vector2d(25, 4))
                .strafeToConstantHeading(new Vector2d(32, 4))
                .build();

        Action ToZone2 = drive.actionBuilder(new Pose2d(32, 4, 0))
                .strafeToConstantHeading(new Vector2d(1, -36))
                .build();

        Action ToBar3 = drive.actionBuilder(new Pose2d(1, -36, 0))
                .strafeToConstantHeading(new Vector2d(25, 6))
                .strafeToConstantHeading(new Vector2d(32, 6))
                .build();

        Action Park = drive.actionBuilder(new Pose2d(32, 6, 0))
                .strafeToConstantHeading(new Vector2d(3, -40))
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

            // -------------------------------------- HOOK 1ST SPEC ------------------------------------
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
                                    Strafe1
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
