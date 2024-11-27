package org.firstinspires.ftc.teamcode.autonomous.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.MecanumDrive;

@Autonomous(name="LeftYellowAutonomous")
public class LeftYellowAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // --------------------------------------- INITIALIZATION ---------------------------------------
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-12, -72, Math.toRadians(270)));
        // put hardware map motors here
        DcMotor motor1 = hardwareMap.get(DcMotor.class, "motor");

        // --------------------------------------- TRAJECTORY ---------------------------------------
        // Declare Trajectory To Bring Robot to the bar
        Action TrajectoryAction1 = drive.actionBuilder(drive.pose) // begin at current pose
                .lineToY(-32)
                .build();

        Action TrajectoryAction2 = drive.actionBuilder(drive.pose)
                .splineTo(new Vector2d(5, 5), Math.toRadians(90))
                .build();

        waitForStart();
        ElapsedTime runtime = new ElapsedTime();

        while (true) {
            if (isStopRequested()) return;

            Actions.runBlocking(
                    new SequentialAction(
                            TrajectoryAction1, // Example of a drive action

                            // This action and the following action do the same thing
                            new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    telemetry.addLine("Action!");
                                    telemetry.update();
                                    return false;
                                }
                            },
                            // Only that this action uses a Lambda expression to reduce complexity
                            (telemetryPacket) -> {
                                telemetry.addLine("Action!");
                                telemetry.update();
                                return false; // Returning true causes the action to run again, returning false causes it to cease
                            },
                            new ParallelAction( // several actions being run in parallel
                                    TrajectoryAction2, // Run second trajectory
                                    (telemetryPacket) -> { // Run some action
                                        motor1.setPower(1);
                                        return false;
                                    }
                            ),
                            drive.actionBuilder(new Pose2d(15, 10, Math.toRadians(125))) // Another way of running a trajectory (not recommended because trajectories take time to build and will slow down your code, always try to build them beforehand)
                                    .splineTo(new Vector2d(25, 15), 0)
                                    .build()

                    )
            );
        }
    }
}
