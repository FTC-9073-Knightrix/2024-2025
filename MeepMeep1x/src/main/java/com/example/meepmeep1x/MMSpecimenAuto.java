package com.example.meepmeep1x;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

public class MMSpecimenAuto {
    static final double forwardAngle = Math.toRadians(90);
    static final double backwardAngle = Math.toRadians(270);
    static final double rightAngle = Math.toRadians(0);
    static final double leftAngle = Math.toRadians(180);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.PI, Math.PI, 15)
                .build();

        // Run meep meep method here
        fiveSpecimenFlippingClaw(myBot);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    // Link to auto that is attempting to be implemented
    // https://www.youtube.com/watch?v=xO0BuFX0f84 and https://www.youtube.com/watch?v=J1zYPewDfEA
    public static void fiveSpecimenFlippingClaw (RoadRunnerBotEntity myBot) {
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(8, -62, forwardAngle))
                        .lineToY(-32)
                        .waitSeconds(0.2)

                        // first u turn
                        .strafeToConstantHeading(new Vector2d(28, -38))
                        .splineToConstantHeading(new Vector2d(29, -38), rightAngle)
                        .splineToConstantHeading(new Vector2d(36, -32), forwardAngle)
                        .splineToConstantHeading(new Vector2d(36, -15), forwardAngle)
                        .splineToConstantHeading(new Vector2d(44, -15), backwardAngle)
                        // down and u turn
                        .splineToConstantHeading(new Vector2d(44, -56), backwardAngle)
                        .splineToConstantHeading(new Vector2d(39, -56), forwardAngle)
                        // go back up and u turn
                        .splineToConstantHeading(new Vector2d(39, -17), forwardAngle)
                        .splineToConstantHeading(new Vector2d(54, -17), backwardAngle)
                        // down and u turn
                        .splineToConstantHeading(new Vector2d(54, -53), backwardAngle)
                        .splineToConstantHeading(new Vector2d(49, -53), forwardAngle)
                        // go back up and u turn
                        .splineToConstantHeading(new Vector2d(49, -15), forwardAngle)
                        .splineToConstantHeading(new Vector2d(61, -15), backwardAngle)
                        // down
                        .splineToConstantHeading(new Vector2d(61, -53), backwardAngle)

                        // pickup 1
                        .strafeToConstantHeading(new Vector2d(40, -58), baseVelConstraint)

                        // TODO HOOK AND PICKUP CYCLES
                 .build());
    }

    public static void fourSpecimenFlippingClaw (RoadRunnerBotEntity myBot) {

    }
}