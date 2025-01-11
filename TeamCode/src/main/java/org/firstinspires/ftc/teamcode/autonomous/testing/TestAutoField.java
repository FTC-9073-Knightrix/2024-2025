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

@Autonomous(name="TEST AUTO FIELD", group = "Autonomous")
public class TestAutoField extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(-62, 4, 0));
        drive.updatePoseEstimate();

        Action ActionOne = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(new Vector2d(-30, 4))
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