package org.firstinspires.ftc.teamcode.autonomous.competition.january;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;
import org.firstinspires.ftc.teamcode.autonomous.PinpointDrive;

import java.util.Arrays;

@Autonomous(name = "twospecimenauto")
public class TwoSpecimenAuto extends FiveSpecimenActions{
    final double forwardAngle = Math.toRadians(90);
    final double backwardAngle = Math.toRadians(270);
    final double rightAngle = Math.toRadians(0);
    final double leftAngle = Math.toRadians(180);

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(4, -62, forwardAngle);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        MecanumDrive drive2 = new MecanumDrive(hardwareMap, beginPose);

        VertLinearMotor vertLinearMotor = new VertLinearMotor(hardwareMap);
        OuttakeClawServo outtakeClawServo = new OuttakeClawServo(hardwareMap);
        OuttakeArmServo outtakeArmServo = new OuttakeArmServo(hardwareMap);
        OuttakeTwistServo outtakeTwistServo = new OuttakeTwistServo(hardwareMap);

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0),
                new AngularVelConstraint(Math.PI / 2)
        ));

        Action ToBarAction1 = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(2, -30), forwardAngle, baseVelConstraint)
                .build();

        Action Park = drive.actionBuilder(new Pose2d(2, -30, forwardAngle))
                .strafeToConstantHeading(new Vector2d(40, -58))
                .build();

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
                                    ToBarAction1,
                                    vertLinearMotor.liftUpToChamber(),
                                    outtakeArmServo.clawArmForward()
                            ),
                            vertLinearMotor.hookOnBar(),
                            outtakeClawServo.openClaw(),
                            new ParallelAction(
                                    outtakeArmServo.clawArmBack(),
                                    Park
                            )
                    )
            );
        }
    }
}
