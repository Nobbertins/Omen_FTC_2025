package com.suman.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 63, 3*Math.PI / 2))
                .lineToY(41)
                .splineToConstantHeading(new Vector2d(-34, 38), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-42, 9), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-42, 53), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-47, 16), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-53, 11), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-53, 49), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-58, 16), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-64, 11), 3*Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-64, 49), 3*Math.PI / 2)
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}