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
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 10)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(10, -66, Math.toRadians(90)))
                .strafeTo(new Vector2d(10,-40))
                .waitSeconds(1.5)
                .strafeTo(new Vector2d(10, -44))
                .strafeTo(new Vector2d(56,-44))
                .turnTo(Math.toRadians(270))
                .strafeTo(new Vector2d(56, -52))
                .waitSeconds(1)
                .strafeTo(new Vector2d(56,-44))
                .turnTo(Math.toRadians(90))
                .strafeTo(new Vector2d(6,-52))
                .waitSeconds(1)
                .strafeTo(new Vector2d(6, -40))
                .waitSeconds(1)
                .strafeTo(new Vector2d(6, -44))
                .strafeTo(new Vector2d(56, -52))


                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}