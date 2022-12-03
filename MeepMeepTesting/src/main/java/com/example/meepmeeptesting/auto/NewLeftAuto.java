package com.example.meepmeeptesting.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class NewLeftAuto {
    public static void main (String args[]){
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -63, 1.5708))
                                .strafeRight(22)
                                .forward(28)
                                .turn(Math.toRadians(-45))
                                .waitSeconds(0.1)
                                /*.UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    Mec.RearArmMid();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    Mec.VertSlideToPos(3, 0.7);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.5, () ->{
                                    Mec.SlideServoOut();
                                })*/
                                .waitSeconds(3)
                                .back(3)
                                .waitSeconds(4)
                                /*.UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                                    Mec.ClawRelease();
                                })*/
                                .back(1)
                                .turn(Math.toRadians(45))
                                .forward(26)
                                .waitSeconds(0.1)
                                /*.UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    Mec.VertSlideToPos(1, 0.8);
                                })*/
                                .turn(Math.toRadians(90))
                                .forward(45)
                                .waitSeconds(0.1)
                                .back(8)
                                .turn(Math.toRadians(90))
                                .forward(15)
                                .turn(Math.toRadians(45))
                                .waitSeconds(0.1)
                                /*.UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    Mec.RearArmMid();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    Mec.VertSlideToPos(3, 0.7);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.5, () ->{
                                    Mec.SlideServoOut();
                                })*/
                                .waitSeconds(3)
                                .back(3)
                                .waitSeconds(4)
                                /*.UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                                    Mec.ClawRelease();
                                })*/
                                .turn(Math.toRadians(45))
                                //.waitSeconds(10)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot)
                .start();
    }
}
