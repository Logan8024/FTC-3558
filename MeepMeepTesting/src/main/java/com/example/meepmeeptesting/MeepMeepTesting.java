package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -66, 0))
                .strafeTo(new Vector2d(0,-56))
                .strafeTo(new Vector2d(0,-62))
                .turnTo(180)
                .strafeTo(new Vector2d(48,-62))
                .waitSeconds(5)
                .strafeTo(new Vector2d(48, -66))
                .turnTo(0)
                .strafeTo(new Vector2d(0,-56))
                .strafeTo(new Vector2d(65,-60))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}