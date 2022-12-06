package com.example.meepmeeptesting.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class PloughingRightAuto {
    public static void main(String args[]) {
        MeepMeep meepMeep = new MeepMeep(650);

        Pose2d StartPose = new Pose2d(35, -72, Math.toRadians(90));

        RoadRunnerBotEntity Bot = new DefaultBotBuilder(meepMeep)
                //region TrajSequence
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(StartPose)
                                .forward(12)
                                .turn(Math.toRadians(90))
                                .lineToSplineHeading(new Pose2d(35, 2, Math.toRadians(180)))
                                .waitSeconds(0.1)
                                .lineToSplineHeading(new Pose2d(35, -15, Math.toRadians(135)))
                                .waitSeconds(0.5)
                                /*.UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                                    Mec.VertSlideToPos(3, 0.7);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                                    Mec.SlideServoOut();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                                    Mec.ClawRelease();
                                })*/
                                .waitSeconds(1)
                                /*.UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                                    Mec.ClawGrip();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                                    Mec.SlideServoIn();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.3, ()-> {
                                    Mec.VertSlideToPos(0, 0.7);
                                })*/
                                .forward(5)
                                .lineToSplineHeading(new Pose2d(56, -12, Math.toRadians(0)))
                                /*.UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                                    Mec.VertSlideToPos(1, 0.7);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                                    Mec.ClawGrip();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                                    Mec.VertSlideUp(0.7);
                                })*/
                                .lineToSplineHeading(new Pose2d(34, -12, Math.toRadians(135)))
                                /*.UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                                    Mec.VertSlideToPos(3, 0.7);
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                                    Mec.SlideServoOut();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                                    Mec.ClawRelease();
                                })*/
                                .waitSeconds(1)
                                /*.UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                                    Mec.ClawGrip();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                                    Mec.SlideServoIn();
                                })
                                .UNSTABLE_addTemporalMarkerOffset(1.3, ()-> {
                                    Mec.VertSlideToPos(0, 0.7);
                                })*/
                                .waitSeconds(10)
                                .build()
                );
        //endregion
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot)
                .start();
    }
}
