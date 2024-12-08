//package org.firstinspires.ftc.teamcode.autonomous;
//
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//@Autonomous(name = "Block Auto", group = "Autonomous")
//public class blockAuto extends AutoMethods{
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initRobot();
//
//        Pose2d beginPose = new Pose2d(-8, -62, Math.toRadians(270));
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8, -62, Math.toRadians(270)));
//        drive.updatePoseEstimate();
//
//        // Drives into bar #1
//        Action action1 = drive.actionBuilder(beginPose)
//                .lineToY(-34)
//                .build();
//        // Drives into bar #1
//        Action action2 = drive.actionBuilder(new Pose2d(-8, -34, Math.toRadians(270)))
//                .lineToY(-32)
//                .build();
//
//        Action action3 = drive.actionBuilder(new Pose2d(-8, -32, Math.toRadians(270)))
//                .lineToY(-44)
//                .build();
//        Action action4 = drive.actionBuilder(new Pose2d(-8, -44, Math.toRadians(180)))
//                .lineToX(-34)
//                .build();
//        Action action5 = drive.actionBuilder(new Pose2d(-34, -44, Math.toRadians(180)))
//                .strafeToConstantHeading(new Vector2d(-36, -25))
//                .build();
//        Action action6 = drive.actionBuilder(new Pose2d(-36, -25, Math.toRadians(180)))
//                .strafeToConstantHeading(new Vector2d(-42, -25))
//                .build();
//        Action action7 = drive.actionBuilder(new Pose2d(-42, -25, Math.toRadians(180)))
//                .turnTo(Math.toRadians(45))
//                .build();
//        Action action8 = drive.actionBuilder(new Pose2d(-42, -25, Math.toRadians(45)))
//                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(45))
//                .build();
//        Action action9 = drive.actionBuilder(new Pose2d(-58, -54, Math.toRadians(45)))
//                .lineToY(-54)
//                .build();
//        Action action10 = drive.actionBuilder(new Pose2d(-58, -54, Math.toRadians(45)))
//                .strafeToLinearHeading(new Vector2d(-56, -23), Math.toRadians(90))
//                .build();
//        Action action11 = drive.actionBuilder(new Pose2d(-56, -23, Math.toRadians(90)))
//                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(45))
//                .build();
//        Action action12 = drive.actionBuilder(new Pose2d(-58, -54, Math.toRadians(45)))
//                .lineToY(-54)
//                .build();
//
//        if (opModeInInit()) {
//            vertLinearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            clawServo.setPosition(0.2);
//            latchServo.setPosition(1.0);
//            armServo.setPosition(0.05);
//        }
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            if (isStopRequested()) return;
//            armServo.setPosition(0.5);
//        }
//
//    }
//}
