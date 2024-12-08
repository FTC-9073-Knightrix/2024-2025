package com.example.meepmeep1x;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import kotlin.math.UMathKt;

public class MeepMeepYSampleRedAllianceAuto {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-9, -62, Math.toRadians(270)))
                .lineToY(-32)
                .lineToY(-44)
                .turnTo(Math.toRadians(180))
                .lineToX(-35)
                .strafeToConstantHeading(new Vector2d(-35, -25))
                .strafeToConstantHeading(new Vector2d(-35, -10))
                                .strafeToConstantHeading(new Vector2d(-26, -10))
//                .strafeToConstantHeading(new Vector2d(-42, -25))
//                .turnTo(Math.toRadians(45))
//                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(45)) // positioning for the dropping
//                .lineToY(-54) // positioning for the dropping
//                .strafeToLinearHeading(new Vector2d(-56, -23), Math.toRadians(90))
//                .strafeToLinearHeading(new Vector2d(-58, -54), Math.toRadians(45)) // positioning for the dropping
//                .lineToY(-54)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}