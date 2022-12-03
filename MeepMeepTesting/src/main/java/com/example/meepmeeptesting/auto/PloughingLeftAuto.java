package com.example.meepmeeptesting.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PloughingLeftAuto {
    public static void main(String args[]) {
        MeepMeep meepMeep = new MeepMeep(650);

        RoadRunnerBotEntity Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -63, 1.5708))
                                .lineTo(new Vector2d(-34,-16))
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
                                .waitSeconds(2)
                                .turn(Math.toRadians(135))
                                .strafeRight(4)
                                .forward(25)
                                .
                                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot)
                .start();
    }
}
