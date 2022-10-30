package org.firstinspires.ftc.teamcode.botbuilders.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomous")
public class BotBuildersRightAuto extends LinearOpMode {

    @Override
    public void runOpMode(){

        BotBuildersMecanumDrive Mec = new BotBuildersMecanumDrive(hardwareMap);
        //region RightAutoTrajSequence
        //Drive.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        TrajectorySequence RightAuto = Mec.trajectorySequenceBuilder(new Pose2d())
                /*---INSERT CODE TO SCAN APRILTAG---*/
                .waitSeconds(2)             //Read AprilTag
                .strafeLeft(30)
                .forward(35)
                .turn(Math.toRadians(-30))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.ClawGrip();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(2, 0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    Mec.SlideServoOut();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    Mec.ClawRelease();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.SlideServoIn();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(0, 0.8);
                })
                .turn(Math.toRadians(30))
                .forward(10)
                .turn(Math.toRadians(90))
                .back(52)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    Mec.CampSlideToPos(1, 0.8);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.IntakeSpeed(1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.CampSlideToPos(0, 0.8);
                    Mec.IntakeSpeed(0);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.IntakeSpeed(-1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.IntakeSpeed(0);
                })
                .forward(20)
                .turn(Math.toRadians(-30))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.ClawGrip();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(2, 0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    Mec.SlideServoOut();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    Mec.ClawRelease();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.SlideServoIn();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(0, 0.8);
                })
                .turn(Math.toRadians(30))
                .back(20)
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    Mec.CampSlideToPos(1, 0.8);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.IntakeSpeed(1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.CampSlideToPos(0, 0.8);
                    Mec.IntakeSpeed(0);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.IntakeSpeed(-1);
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.IntakeSpeed(0);
                })
                .forward(20)
                .turn(Math.toRadians(-30))  //Deliver Cone3
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.ClawGrip();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(2, 0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () ->{
                    Mec.SlideServoOut();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    Mec.ClawRelease();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.SlideServoIn();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(0, 0.8);
                })
                .turn(Math.toRadians(30))
                .strafeLeft(23)            //Park in Zone2
                /*---INSERT CODE TO PARK---*/
                .build();
        //endregion

        Mec.ResetVertSlideEncoders();
        Mec.ResetCampSlideEncoders();

        Mec.WriteData(telemetry);

        waitForStart();

        Mec.followTrajectorySequence(RightAuto);
    }
}
