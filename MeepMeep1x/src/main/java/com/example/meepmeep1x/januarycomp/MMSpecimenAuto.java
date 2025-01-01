package com.example.meepmeep1x.januarycomp;

import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(4, -62, forwardAngle))
                        .splineToConstantHeading(new Vector2d(4, -30), forwardAngle)

                        // first u turn
                        .strafeToConstantHeading(new Vector2d(28, -36))
                        .splineToConstantHeading(new Vector2d(29, -36), rightAngle)
                        .splineToConstantHeading(new Vector2d(36, -32), forwardAngle)
                        .splineToConstantHeading(new Vector2d(36, -17), forwardAngle)
                        .splineToConstantHeading(new Vector2d(44, -17), backwardAngle)
                        // down and u turn
                        .splineToConstantHeading(new Vector2d(44, -53), backwardAngle)
                        .splineToConstantHeading(new Vector2d(39, -53), forwardAngle)
                        // go back up and u turn
                        .splineToConstantHeading(new Vector2d(39, -17), forwardAngle)
                        .splineToConstantHeading(new Vector2d(54, -17), backwardAngle)
                        // down and u turn
                        .splineToConstantHeading(new Vector2d(54, -53), backwardAngle)
                        .splineToConstantHeading(new Vector2d(49, -53), forwardAngle)
                        // go back up and u turn
                        .splineToConstantHeading(new Vector2d(49, -17), forwardAngle)
                        .splineToConstantHeading(new Vector2d(61, -17), backwardAngle)
                        // down
                        .splineToConstantHeading(new Vector2d(61, -53), backwardAngle)

                        // pickup 1


                .strafeToConstantHeading(new Vector2d(40, -58), baseVelConstraint)
                .splineToConstantHeading(new Vector2d(40, -57), forwardAngle)
                .splineToConstantHeading(new Vector2d(6, -31), forwardAngle)
                .strafeToConstantHeading(new Vector2d(40, -58))
                .splineToConstantHeading(new Vector2d(40, -57), forwardAngle)
                .splineToConstantHeading(new Vector2d(4, -31), forwardAngle)
                .strafeToConstantHeading(new Vector2d(40, -58))
                                .strafeToConstantHeading(new Vector2d(2, -31))
                                .strafeToConstantHeading(new Vector2d(40, -58))
                                .strafeToConstantHeading(new Vector2d(0, -31))
                                .strafeToLinearHeading(new Vector2d(22, -47), Math.toRadians(330))
                /*   .strafeToConstantHeading(new Vector2d(6, -31))
                    .strafeToConstantHeading(new Vector2d(40, -58))
                .strafeToConstantHeading(new Vector2d(4, -31))
                .strafeToConstantHeading(new Vector2d(40, -58))
                .strafeToConstantHeading(new Vector2d(2, -31))
*/
                // TODO HOOK AND PICKUP CYCLES
                 .build());
    }

    public static void fourSpecimenFlippingClaw (RoadRunnerBotEntity myBot) {

    }
}