package com.example.meepmeeptesting.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class NewRightAuto {
    public static void main (String args[]){
        MeepMeep meepMeep = new MeepMeep(650);

        Pose2d DeliveryPose2d = new Pose2d(12,-12, Math.toRadians(45));

        RoadRunnerBotEntity Bot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(34, -63, 1.5708))
                                .waitSeconds(10)
                                .strafeLeft(25)
                                /*.UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                                    Mec.VertSlideToPos(3, 0.7);
                                })*/
                                .forward(15)
                                .turn(Math.toRadians(25))
                                /*.UNSTABLE_addTemporalMarkerOffset(3.5, ()-> {
                                    Mec.ClawRelease();
                                    sleep(250);
                                    Mec.VertSlideToPos(0,0.7);
                                })*/
                                .waitSeconds(3.5) //wait for the slides to catch up
                                .turn(Math.toRadians(-25))
                                .back(2)
                                .forward(32)
                                .turn(Math.toRadians(-90))
                                .forward(36)
                                /*.UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                                    Mec.SlideServoAutoPickUp();
                                })*/
                                .forward(4)
                                /*.UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                                    Mec.ClawGrip();
                                    sleep(350);
                                    Mec.VertSlideToAutoPickupPos();
                                })*/
                                .forward(1)
                                .waitSeconds(2)
                                .back(20)
                                .turn(Math.toRadians(-40))
                                /*.UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                                    sleep(250);
                                    Mec.ClawRelease();
                                })*/
                                .waitSeconds(1.5)
                                /*.UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                                    Mec.ClawGrip();
                                })*/
                                .turn(Math.toRadians(35))
                                /*.UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                                    Mec.SlideServoOut();
                                    Mec.RotateClaw(0);
                                    sleep(1000);
                                    Mec.SlideServoToPos(0);
                                    sleep(250);
                                    Mec.VertSlideToPos(0, 0.7);
                                })*/
                                .waitSeconds(2)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(Bot)
                .start();
    }
}
