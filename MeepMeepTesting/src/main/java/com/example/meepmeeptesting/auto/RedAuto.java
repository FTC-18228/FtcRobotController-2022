package com.example.meepmeeptesting.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class RedAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -70, 1.5708))
                                .waitSeconds(2)             //Read AprilTag
                                .strafeRight(24)
                                .forward(35)
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1)             //Deliver Preload
                                .turn(Math.toRadians(45))
                                .forward(23)
                                .turn(Math.toRadians(-90))
                                .back(50)
                                .waitSeconds(1)             //Pickup Cone2
                                .forward(50)
                                .turn(Math.toRadians(-45))
                                .waitSeconds(1)             //Deliver Cone2
                                .turn(Math.toRadians(45))
                                .back(50)
                                .waitSeconds(1)             //Pickup Cone3
                                .forward(50)
                                .turn(Math.toRadians(-45))  //Deliver Cone3
                                .waitSeconds(1)
                                .turn(Math.toRadians(45))
                                .strafeRight(23)            //Park in Zone2
                                /*---INSERT CODE TO PARK---*/
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}