package com.example.meepmeep1x;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepBlueSpecimenAuto {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 15)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-8, 62, Math.toRadians(90)))
                .lineToY(32)
                .splineToConstantHeading(new Vector2d(-8, 33), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-46, 58, Math.toRadians(270)), Math.toRadians(90))
                .lineToY(62)

                .splineToConstantHeading(new Vector2d(-46, 61), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(-4, 32, Math.toRadians(90)), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(-4, 34), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d( -35, 34), Math.toRadians(270))

                .splineToConstantHeading(new Vector2d(-35, 15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44, 15), Math.toRadians(90))
                .lineToY(55)

                .splineToConstantHeading(new Vector2d(-44, 15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-56, 15), Math.toRadians(90))
                .lineToY(55)

                .splineToConstantHeading(new Vector2d(-56, 15), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-60, 15), Math.toRadians(90))
                .lineToY(55)

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}