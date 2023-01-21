package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
        public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(400);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(30, 30, Math.toRadians(180), Math.toRadians(180), 13.79)
                    .followTrajectorySequence(drive ->
                                    drive.trajectorySequenceBuilder(new Pose2d(-26.37, -2.1, Math.toRadians(45)))
                                    .splineToConstantHeading(new Vector2d(-32.37, -12.3), Math.toRadians(45))
                                    //.splineTo(new Vector2d(-24.37, -19.03), Math.toRadians(87.0))
                                            .turn(Math.toRadians(135))
                                    .splineToConstantHeading(new Vector2d(-62.10, -12.3), Math.toRadians(180))
                                    .build()
                    //.splineToConstantHeading()
                            /*
                            drive.trajectorySequenceBuilder(new Pose2d(-35.50, -60.20, Math.toRadians(90.00)))
                                    //.splineToConstantHeading()
                                    .splineTo(new Vector2d(-37.28, -19.03), Math.toRadians(87.56))
                                    .splineTo(new Vector2d(-26.37, -2.1), Math.toRadians(45))
                                    .build()

                             */
                    );

            meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                    .setDarkMode(true)
                    .setBackgroundAlpha(0.95f)
                    .addEntity(myBot)
                    .start();
        }
    }