package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepExample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180),0.5 )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -58, (3.14/2)))
                                .forward(15)
                                .waitSeconds(0.5)
                                .forward(31)
                                .turn(-3.14/2)
                                .forward(15)
                                .waitSeconds(0.5)
                                .forward(-25)
                                .turn(-Math.toRadians(75))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(75))
                                .forward(24)
                                .waitSeconds(0.5)
                                .forward(-24)
                                .turn(-Math.toRadians(75))
                                .waitSeconds(0.5)
                                .turn(Math.toRadians(75))
                                .forward(-15)
                                .strafeRight(24)




                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}