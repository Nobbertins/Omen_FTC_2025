package com.suman.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private static final Pose2d STARTING_POSE = new Pose2d(-64, -8, Math.toRadians(180));

    // Sample positions (adjust these based on your field measurements)
    private static final Vector2d SPECIMEN_DROP = new Vector2d(-36, -8);
    private static final Vector2d afterSPECIMEN_DROP = new Vector2d(-51, -34);
    private static final Vector2d SAMPLE_1_PRE = new Vector2d(-34, -44);
    private static final Vector2d SAMPLE_1= new Vector2d(-15, -53);
    private static final Vector2d SAMPLE_2_PRE = new Vector2d(-15, -52);
    private static final Vector2d SAMPLE_2 = new Vector2d(-15, -60);
    private static final Vector2d SAMPLE_3_PRE = new Vector2d(-15, -59);
    private static final Vector2d SAMPLE_3 = new Vector2d(-15, -64);
    private static final Vector2d HUMAN_ZONE_1 = new Vector2d(-59, -53);
    private static final Vector2d HUMAN_ZONE_2 = new Vector2d(-59, -60);
    private static final Vector2d HUMAN_ZONE_3 = new Vector2d(-59, -64);
    private static final Vector2d BUCKET_POS = new Vector2d(10, -12);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(STARTING_POSE)
                .strafeTo(SPECIMEN_DROP)
                .strafeToLinearHeading(afterSPECIMEN_DROP, Math.toRadians(0))
                .splineToConstantHeading(SAMPLE_1_PRE, Math.toRadians(0))
                // Move to first sample
                .splineToConstantHeading(SAMPLE_1, Math.toRadians(0))

                // Move to bucket
                .strafeTo(HUMAN_ZONE_1)
                // Move to second sample
                .strafeToConstantHeading(SAMPLE_2_PRE)
                .strafeTo(SAMPLE_2) // Time for intake

                .strafeTo(HUMAN_ZONE_2)
                // Back to bucket


                .strafeToConstantHeading(SAMPLE_3_PRE)
                .strafeTo(SAMPLE_3)
                .strafeTo(HUMAN_ZONE_3)

                .waitSeconds(5)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}