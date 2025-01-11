package org.firstinspires.ftc.teamcode.autonomous.testing;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.PinpointDrive;

@Autonomous(name="TEST AUTO ZERO", group = "Autonomous")
public class TestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(90)));
        drive.pinpoint.setPosition(new Pose2d(0, 0, Math.toRadians(90)));

        Action ActionOne = drive.actionBuilder(new Pose2d(0, 0, Math.toRadians(90)))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(10, 0))
                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(0, 0))
                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(-10, 0))
                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(0, 0))
                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(0, 10))
                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(0, 0))
                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(0, -10))
                .waitSeconds(0.5)

                .strafeToConstantHeading(new Vector2d(0, -10))

                .build();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("pose",  drive.pose);
            telemetry.update();
        }
        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                    new SequentialAction(
                            ActionOne
                    )
                )
        );
        telemetry.addData("pose",  drive.pose);
    }
}


