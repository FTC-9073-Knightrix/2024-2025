package com.example.MeepMeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(12, -72, Math.toRadians(270)))
                        .forward(-40)
                        .splineToLinearHeading(new Pose2d(37, -25, Math.toRadians(0)), Math.toRadians(45))
                        .waitSeconds(2)

                        .lineToLinearHeading(new Pose2d(48, -52, Math.toRadians(300))) // return to bucket
                        .waitSeconds(1.5)

                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(47, -25, Math.toRadians(0)), Math.toRadians(0))
                        .waitSeconds(2)

                        .lineToLinearHeading(new Pose2d(48, -52, Math.toRadians(300))) // return to bucket
                        .waitSeconds(1.5)

                        .splineToLinearHeading(new Pose2d(58, -25, Math.toRadians(0)), Math.toRadians(0))
                        .setReversed(false)
                        .waitSeconds(2)

                        .lineToLinearHeading(new Pose2d(48, -52, Math.toRadians(300))) // return to bucket
                        .waitSeconds(1.5)

                        .lineToLinearHeading(new Pose2d(34, -11, Math.toRadians(0))) // go to bar to ascend
                        .waitSeconds(0.25)
                        .back(10)
                        .waitSeconds(1)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}