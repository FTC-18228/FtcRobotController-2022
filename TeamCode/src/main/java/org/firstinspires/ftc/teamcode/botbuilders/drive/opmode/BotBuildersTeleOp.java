package org.firstinspires.ftc.teamcode.botbuilders.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;


import org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;

@Config
@TeleOp(group = "drive")
public class BotBuildersTeleOp extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        //mecDrive.reAlignIMU();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);


        boolean clawGripState = true;
        boolean rearArmState = true;
        boolean slideArmState = false;
        boolean clawRotateState = false;

        ElapsedTime timer = new ElapsedTime();
        double velocity = 0.5;

        timer.startTime();


        Pose2d poseEstimate = mecDrive.getPoseEstimate();

        //TrajectorySequence builtAutoPath = BuildPath(mecDrive);
        mecDrive.SlideServoIn();

        mecDrive.RearArmMid();
        waitForStart();
        while (!isStopRequested()) { // while robot is running and stop button is not pressed

            gp1.readButtons();
            gp2.readButtons();

           // mecDrive.WriteData(telemetry);

            if(gamepad1.a && gamepad1.b){
                mecDrive.ReAlignIMU();
                telemetry.addData("IMU", "RESET");
                telemetry.update();
            }

            poseEstimate = mecDrive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * velocity,
                    -gamepad1.left_stick_x * velocity

            ).rotated(-poseEstimate.getHeading());

            if (gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                input = new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x).rotated(-poseEstimate.getHeading());

            }

            Pose2d vel = new Pose2d(
                    input.getX(),
                    input.getY(),
                    -gamepad1.right_stick_x * velocity
            );

            mecDrive.setWeightedDrivePower(vel);
            mecDrive.update();


            if (gamepad1.a || gamepad2.a) {
                mecDrive.IntakeSpeed(1);
                telemetry.addData("Intake", "On");
                telemetry.update();
            } else if (gamepad1.b || gamepad2.b) {
                mecDrive.IntakeSpeed(-1);
                telemetry.addData("Intake", "Rev");
                telemetry.update();

            } else {
                mecDrive.IntakeSpeed(0);
                telemetry.addData("Intake", "Off");
                telemetry.update();
            }

            if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 || gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
                //fire the load up automation
                //ensure the camp slide is in

                 vel = new Pose2d(
                        0,
                        0,
                        -gamepad1.right_stick_x * velocity
                );

                mecDrive.setWeightedDrivePower(vel);
                mecDrive.update();

                mecDrive.SlideServoIn();
                mecDrive.IntakeSlideDown();
                mecDrive.IntakeSpeed(1.0);
                //mecDrive.CampSlideToPos(0, 0.5);
                //need to wait for the slide to move
                //mecDrive.CampSlideDelay(this);
                //move the linear slide arm in
                //mecDrive.SlideServoIn();
                //first make sure the claw is open
                mecDrive.ClawRelease();

                sleep(250);

                mecDrive.RearArmIn();
                sleep(600);
                mecDrive.IntakeSpeed(-1.0);
                sleep(500);
                mecDrive.RearArmOut();
                sleep(150);
                mecDrive.SlideServoPickUp();
                sleep(150);
                mecDrive.ClawGrip();
                sleep(100);
                mecDrive.IntakeSlideUp();

                clawGripState = true;
                sleep(500);
               // mecDrive.SlideServoIn();

            }

            //slideArmState - X Button
            if(gp1.wasJustPressed(GamepadKeys.Button.X) || gp2.wasJustPressed(GamepadKeys.Button.X)){
                slideArmState = !slideArmState;
                if(slideArmState){
                    mecDrive.ClawGrip();
                    clawGripState = true;
                    sleep(250);
                    mecDrive.SlideServoIn();
                    telemetry.addData("Slide Arm", "In");
                }else{
                    mecDrive.ClawGrip();
                    clawGripState = true;
                    sleep(250);
                    mecDrive.SlideServoOut();

                    telemetry.addData("Slide Arm", "Out");
                }
            }

            //left bumper toggles the state of the claw - LEFT_BUMPER
            //if(gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) || gp2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){}

            if(gp1.wasJustPressed(GamepadKeys.Button.Y)){

                clawGripState = !clawGripState;
                if(clawGripState){
                    mecDrive.ClawRelease();
                }else{
                    mecDrive.ClawGrip();
                }
            }

            if(gp2.wasJustPressed(GamepadKeys.Button.Y)){
                clawRotateState = !clawRotateState;
                if(clawRotateState){
                    mecDrive.ClawGrip();
                    mecDrive.VertSlideToPos(2, 0.7);
                    mecDrive.SlideServoToPos(0.75);
                    sleep(500);
                    mecDrive.RotateClaw(1);
                    sleep(1000);
                    mecDrive.SlideServoToPos(0);
                    mecDrive.VertSlideToPos(1, 0.7);
                    mecDrive.RearArmMid();
                }else {
                    mecDrive.ClawGrip();
                    mecDrive.VertSlideToPos(2, 0.7);
                    mecDrive.SlideServoToPos(0.75);
                    sleep(500);
                    mecDrive.RotateClaw(0);
                    sleep(1000);
                    mecDrive.SlideServoToPos(0);
                    mecDrive.VertSlideToPos(1, 0.7);
                    }
                }

            //left bumper toggles the state of the rear arm
            if(gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || gp2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                rearArmState = !rearArmState;
                if(rearArmState){
                    mecDrive.RearArmIn();
                    telemetry.addData("ARM", "In");
                }else{
                    mecDrive.RearArmOut();
                    telemetry.addData("ARM", "Out");
                }
            }

            if(gp1.gamepad.dpad_up){
                mecDrive.VertSlideUp(0.7);
            }else if(gp1.gamepad.dpad_down){
                mecDrive.VertSlideDown(0.7);
            }else{
                mecDrive.VertSlideUp(0);
            }

            if(gp2.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)){
                mecDrive.RearArmDownIncr();
            }else if (gp2.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
                mecDrive.RearArmUpIncr();
            }

            if(gp2.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)){
                mecDrive.RearArmOut();
            }

            if(gp1.gamepad.dpad_left){
                //mecDrive.CampSlideIn(0.5);
                mecDrive.RearArmOut();
                rearArmState = false;
            }else if(gp1.gamepad.dpad_right){
                //mecDrive.CampSlideOut(0.5);
                mecDrive.RearArmIn();
                rearArmState = true;
            }else{
               // mecDrive.CampSlideOut(0);
            }

            telemetry.update();


        }
    }

}


