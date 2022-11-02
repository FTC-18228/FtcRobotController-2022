package org.firstinspires.ftc.teamcode.botbuilders.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.demo.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "autonomous")
public class BotBuildersRightAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AprilTagDetection tagOfInterest = null;

    private int POS_1_TAG_ID = 1;
    private int POS_2_TAG_ID = 2;
    private int POS_3_TAG_ID = 3;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    @Override
    public void runOpMode(){
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        BotBuildersMecanumDrive Mec = new BotBuildersMecanumDrive(hardwareMap);
        //region RightAutoTrajSequence
        //Drive.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        Mec.setPoseEstimate(new Pose2d(0, 0));

        TrajectorySequence RightAuto = Mec.trajectorySequenceBuilder(new Pose2d())

                .strafeLeft(22)
                .forward(28)
                .turn(Math.toRadians(-45))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.RearArmMid();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    Mec.VertSlideToPos(2, 0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () ->{
                    Mec.SlideServoOut();
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-5))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    Mec.ClawRelease();
                })
                .turn(Math.toRadians(-5))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Mec.ClawGrip();

                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {

                    Mec.SlideServoIn();

                })
                .turn(Math.toRadians(55))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(0, 0.8);
                })
                .forward(18)

                /*.back(52)
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

        TrajectorySequence Pos1Park = Mec.trajectorySequenceBuilder(RightAuto.end())
                .strafeRight(1)
                .build();


        TrajectorySequence Pos2Park = Mec.trajectorySequenceBuilder(RightAuto.end())
                .strafeRight(28)
                .build();

        TrajectorySequence Pos3Park = Mec.trajectorySequenceBuilder(RightAuto.end())
                .strafeRight(55)
                .build();

        //Mec.WriteData(telemetry);
        Mec.SlideServoPickUp();
        Mec.ClawGrip();
        Mec.RearArmMid();

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == POS_1_TAG_ID)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if(tag.id == POS_2_TAG_ID)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else if(tag.id == POS_3_TAG_ID)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                    else{
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {

                    telemetry.addData("ID:", tagOfInterest.id );
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        telemetry.addData("ID:", tagOfInterest.id );
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    telemetry.addData("ID:", tagOfInterest.id );
                }

            }

            telemetry.update();
            sleep(20);
        }

        waitForStart();

        Mec.followTrajectorySequence(RightAuto);

        if(tagOfInterest != null){

            if(tagOfInterest.id == POS_1_TAG_ID){

                //No need to move


            }else if(tagOfInterest.id == POS_2_TAG_ID){

                Mec.followTrajectorySequence(Pos2Park);
            }
            else if(tagOfInterest.id == POS_3_TAG_ID){

                Mec.followTrajectorySequence(Pos3Park);
            }

        }else{
            //park anywhere, 1 in 3 chance?
            Mec.followTrajectorySequence(Pos1Park);
        }
    }
}
