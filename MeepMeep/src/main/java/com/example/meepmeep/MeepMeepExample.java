package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepExample {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity autored1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180),0.5 )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
                                .lineToConstantHeading(new Vector2d(35, -2))

                                .build()
                );

        RoadRunnerBotEntity autored2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeRedDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180),0.5 )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -60, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(-37,-12, Math.toRadians(200)))
//                                .strafeTo(new Vector2d(-30, 2))

//                                .lineToConstantHeading(new Vector2d(-35,-12))
//                                .strafeTo(new Vector2d(-23, -12))
//                                .waitSeconds(3)
//                                .strafeTo(new Vector2d(-12, -12))
//                                .strafeTo(new Vector2d(-60, -12))
//                                .lineToConstantHeading(new Vector2d(35, -2))



                                //or  .lineToLinearHeading(new Pose2d(36,0, Math.toRadians(-13)))

                                .lineToSplineHeading(new Pose2d(35, -2, Math.toRadians(-20)))






                                .build()
                );

        RoadRunnerBotEntity autoblue1 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueDark())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180),0.5 )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, 60, Math.toRadians(270)))
//                                .lineToConstantHeading(new Vector2d(35, 34))
//                                .strafeTo(new Vector2d(12, 34))
                                .lineToConstantHeading(new Vector2d(35,12))
                                .strafeTo(new Vector2d(23, 12))
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(60, 12))
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(12, 12))
                                .waitSeconds(1)
                                .strafeTo(new Vector2d(35,12))





                                //or  .lineToLinearHeading(new Pose2d(36,0, Math.toRadians(-13)))






                                .build()
                );

        RoadRunnerBotEntity autoblue2 = new DefaultBotBuilder(meepMeep)
                .setColorScheme(new ColorSchemeBlueLight())
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(180), Math.toRadians(180),0.5 )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(90)))
//                                .lineToConstantHeading(new Vector2d(-35, 34))
//                                .strafeTo(new Vector2d(-12, 34))
                                .lineToConstantHeading(new Vector2d(-35,12))
                                .strafeTo(new Vector2d(-23, 12))
                                .waitSeconds(3)
                                .strafeTo(new Vector2d(-12, 12))



                                //or  .lineToLinearHeading(new Pose2d(36,0, Math.toRadians(-13)))






                                .build()
                );


        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //.addEntity(autored1)
                .addEntity(autored2)
                //.addEntity(autoblue1)
                //.addEntity(autoblue2)
                .start();
    }

}