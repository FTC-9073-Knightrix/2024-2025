package com.example.meepmeep1x.januarycomp;

import com.acmerobotics.roadrunner.Action;
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
                .setDimensions(17.5, 18)
                .setConstraints(50, 50, Math.PI, Math.PI, 15)
                .build();

        // Run meep meep method here
//        fiveSpecimenFlippingClaw(myBot);
//        fourSpecimenFlippingClawField(myBot);
        fourSpecimenFlippingClawZerod(myBot);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    // Link to auto that is attempting to be implemented
    // https://www.youtube.com/watch?v=xO0BuFX0f84 and https://www.youtube.com/watch?v=J1zYPewDfEA
    public static void fiveSpecimenFlippingClaw (RoadRunnerBotEntity myBot)
    {
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


                .strafeToConstantHeading(new Vector2d(40, -58))
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

    // Subtract 90 degrees from field heading
    // Add 62 to y, flip sign of x then add 4 to x
    // Then, flip x and y
    public static void fourSpecimenFlippingClawField (RoadRunnerBotEntity myBot) {
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(4, -62, Math.toRadians(90)))
                // To bar 1
                .splineToConstantHeading(new Vector2d(4, -30), Math.toRadians(90))

                // Curve out
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(36, -30), Math.toRadians(90))

                // Straight forward
//                .strafeToConstantHeading(new Vector2d(35.5, -20))

                // ( curve
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(39, -12), Math.toRadians(0))
                // ) curve
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(45, -20), Math.toRadians(270))
                // Straight back
                .strafeToConstantHeading(new Vector2d(45, -56))

                // Straight forward 2
                .strafeToConstantHeading(new Vector2d(45, -20))

                // ( curve 2
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50, -12), Math.toRadians(0))
                // ) curve 2
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -20), Math.toRadians(270))

                // Straight back 2
                .strafeToConstantHeading(new Vector2d(55, -56))

                // To Left
                .strafeToConstantHeading(new Vector2d(40, -56))

                // Back into zone
                .strafeToConstantHeading(new Vector2d(40, -61))

                // To bar 2
                .strafeToConstantHeading(new Vector2d(1, -45))
                .strafeToConstantHeading(new Vector2d(1, -30))

                // Back To zone
                .strafeToConstantHeading(new Vector2d(40, -56))
                .strafeToConstantHeading(new Vector2d(40, -61))

                // To bar 3
                .strafeToConstantHeading(new Vector2d(-2, -45))
                .strafeToConstantHeading(new Vector2d(-2, -30))

                // Back To zone
                .strafeToConstantHeading(new Vector2d(40, -56))
                .strafeToConstantHeading(new Vector2d(40, -61))

                // To bar 4
                .strafeToConstantHeading(new Vector2d(-4, -45))
                .strafeToConstantHeading(new Vector2d(-4, -30))

                // Back To zone
                .strafeToConstantHeading(new Vector2d(43, -56))
                .build()
        );
    }
    public static void fourSpecimenFlippingClawZerod (RoadRunnerBotEntity myBot) {
        // - 4 x, + 62 y
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                // To Bar 1
                .splineToConstantHeading(new Vector2d(32, 0), 0)

                // Curve out
                .setTangent(Math.toRadians(180))

                .splineToConstantHeading(new Vector2d(32, -32), Math.toRadians(0))
                // Straight forward
                .strafeToConstantHeading(new Vector2d(42, -32))

                // ( curve
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(50, -36), Math.toRadians(270))
                // ) curve
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -42), Math.toRadians(180))
                // Straight back
                .strafeToConstantHeading(new Vector2d(6, -42))


                // Straight forward 2
                .strafeToConstantHeading(new Vector2d(42, -42))

                // ( curve 2
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -47), Math.toRadians(270))
                // ) curve 2
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -52), Math.toRadians(180))

                // Straight back 2
                .strafeToConstantHeading(new Vector2d(6, -52))
                // Straight forward 3
                .strafeToConstantHeading(new Vector2d(42, -52))
                // ( curve 3
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(48, -54.5), Math.toRadians(270))
                // ) curve 3
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(42, -56), Math.toRadians(180))

                // Straight back 3
                .strafeToConstantHeading(new Vector2d(15, -56))

                // Curve into the zone
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(2, -36), Math.toRadians(120))

                .strafeToConstantHeading(new Vector2d(24, -4))

                .splineToConstantHeading(new Vector2d(31, 2.5), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(32, 2.5), Math.toRadians(0))

                .strafeToConstantHeading(new Vector2d(2, -36))

                .strafeToConstantHeading(new Vector2d(32, 4.5))

                .strafeToConstantHeading(new Vector2d(2, -36))

                .strafeToConstantHeading(new Vector2d(32, 6.6))

                .strafeToConstantHeading(new Vector2d(2, -36))

                .strafeToConstantHeading(new Vector2d(32, 8.5))

                .strafeToConstantHeading(new Vector2d(6, -40))
                .build()
        );
    }
}
