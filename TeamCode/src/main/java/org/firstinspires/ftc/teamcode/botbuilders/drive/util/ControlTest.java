package org.firstinspires.ftc.teamcode.botbuilders.drive.util;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersMecanumDrive;

@Config
@TeleOp(group = "debug")
@Disabled
public class ControlTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive Mec = new BotBuildersMecanumDrive(hardwareMap);

        GamepadEx Controller1 = new GamepadEx(gamepad1);
        GamepadEx Controller2 = new GamepadEx(gamepad2);

        Pose2d PoseEst = Mec.getPoseEstimate();
        double velocity = 0.5;

        waitForStart();
        while(!isStopRequested()){

            Controller1.readButtons();
            Controller2.readButtons();

            Vector2d Input = new Vector2d(
                    -gamepad1.left_stick_y * velocity,
                    -gamepad1.left_stick_x * velocity
            ).rotated(-PoseEst.getHeading());

            if(Controller1.isDown(GamepadKeys.Button.X)){
                Input = new Vector2d(
                        gamepad1.left_stick_y,
                        gamepad1.left_stick_x
                ).rotated(-PoseEst.getHeading());
            }
            Pose2d vel = new Pose2d(
                    Input.getX(),
                    Input.getY(),
                    -gamepad1.right_stick_x * velocity
            );

            Mec.setWeightedDrivePower(vel);
            Mec.update();
        }
    }
}
