package com.example.meepmeeptesting.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueAuto {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34, 70, Math.toRadians(-90)))
                                /*---INSERT CODE TO SCAN APRILTAG---*/
                                .waitSeconds(2)             //Read AprilTag
                                .strafeRight(30)
                                .forward(35)
                                .turn(Math.toRadians(30))
                                .waitSeconds(1)             //Deliver Preload
                                .turn(Math.toRadians(-30))
                                .forward(10)
                                .turn(Math.toRadians(-90))
                                .back(52)
                                .waitSeconds(1)             //Pickup Cone2
                                .forward(20)
                                .turn(Math.toRadians(30))
                                .waitSeconds(1)             //Deliver Cone2
                                .turn(Math.toRadians(-30))
                                .back(20)
                                .waitSeconds(1)             //Pickup Cone3
                                .forward(20)
                                .turn(Math.toRadians(30))  //Deliver Cone3
                                .waitSeconds(1)
                                .turn(Math.toRadians(-30))
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