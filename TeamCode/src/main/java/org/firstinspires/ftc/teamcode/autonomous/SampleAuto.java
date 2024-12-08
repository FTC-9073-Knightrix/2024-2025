package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SampleAuto extends AutoMethods{

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();

        Pose2d beginPose = new Pose2d(-8, -62, Math.toRadians(270));
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8, -62, Math.toRadians(270)));
        drive.updatePoseEstimate();

        // Drives into bar #1
        Action action1 = drive.actionBuilder(beginPose)
                .lineToY(-34)
                .build();
        // Drives into bar #1
        Action action2 = drive.actionBuilder(new Pose2d(-8, -34, Math.toRadians(270)))
                .lineToY(-32)
                .build();

        if (opModeInInit()) {
            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            clawServo.setPosition(0.2);
            latchServo.setPosition(1.0);
            armServo.setPosition(0.05);
        }

        waitForStart();

        if (opModeIsActive()) {
            if (isStopRequested()) return;
            armServo.setPosition(0.5);
        }

    }
}
