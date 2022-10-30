package org.firstinspires.ftc.teamcode.botbuilders.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonomous")
public class BotBuildersLeftAuto extends LinearOpMode {
    @Override
    public void runOpMode(){

        BotBuildersMecanumDrive Mec = new BotBuildersMecanumDrive(hardwareMap);
        //region LeftAutoTrajSequence
        TrajectorySequence LeftAuto = Mec.trajectorySequenceBuilder(new Pose2d())
                /*---INSERT CODE TO SCAN APRILTAG---*/
                .waitSeconds(2)             //Read AprilTag
                .strafeRight(32)
                .forward(30)
                .turn(Math.toRadians(30))
                .waitSeconds(1)             //Deliver Preload
                .turn(Math.toRadians(-30))
                .forward(15)
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
                .turn(Math.toRadians(30))   //Deliver Cone3
                .waitSeconds(1)
                .turn(Math.toRadians(-30))
                .strafeRight(23)            //Park in Zone2
                /*---INSERT CODE TO PARK---*/
                .build();
        //endregion

        Mec.ResetVertSlideEncoders();
        Mec.ResetCampSlideEncoders();

        Mec.WriteData(telemetry);

        waitForStart();

        Mec.followTrajectorySequence(LeftAuto);

    }
}
