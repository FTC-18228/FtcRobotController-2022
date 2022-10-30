package org.firstinspires.ftc.teamcode.botbuilders.drive.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "debug")
@Disabled
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive Mec = new BotBuildersMecanumDrive(hardwareMap);
        waitForStart();

        TrajectorySequence TestAuto = Mec.trajectorySequenceBuilder(new Pose2d())
                .forward(2)
                .waitSeconds(0.2)
                .strafeLeft(2)
                .waitSeconds(0.2)
                .back(2)
                .waitSeconds(0.2)
                .strafeRight(2)
                .build();
        Mec.followTrajectorySequence(TestAuto);
    }
}
