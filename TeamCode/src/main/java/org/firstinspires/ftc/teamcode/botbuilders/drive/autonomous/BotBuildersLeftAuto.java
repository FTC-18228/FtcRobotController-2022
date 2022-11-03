package org.firstinspires.ftc.teamcode.botbuilders.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
public class BotBuildersLeftAuto extends LinearOpMode {

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
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        BotBuildersMecanumDrive Mec = new BotBuildersMecanumDrive(hardwareMap);
        //region LeftAutoTrajSequence
        //working on
        TrajectorySequence LeftAuto = Mec.trajectorySequenceBuilder(new Pose2d())
                .strafeRight(30)
                .forward(28)
                .turn(Math.toRadians(-45))
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.RearArmMid();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    Mec.VertSlideToPos(3, 0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.5, () ->{
                    Mec.SlideServoOut();
                })
                .waitSeconds(1)
                .turn(Math.toRadians(-5))
                .waitSeconds(2)
                .back(3)
                .waitSeconds(4)
                .UNSTABLE_addTemporalMarkerOffset(0.2, ()-> {
                    Mec.ClawRelease();
                })
                .back(1)
                .waitSeconds(1)
                .turn(Math.toRadians(5))
                .forward(1)
                .strafeRight(2)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    Mec.ClawGrip();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    Mec.SlideServoIn();
                })
                .turn(Math.toRadians(40))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    Mec.VertSlideToPos(0, 0.8);
                })
                .forward(22)
                .build();
        //endregion

        TrajectorySequence Pos1Park = Mec.trajectorySequenceBuilder(LeftAuto.end())
                .strafeLeft(55)
                .build();


        TrajectorySequence Pos2Park = Mec.trajectorySequenceBuilder(LeftAuto.end())
                .strafeLeft(28)
                .build();

        TrajectorySequence Pos3Park = Mec.trajectorySequenceBuilder(LeftAuto.end())
                .strafeLeft(1)
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

        Mec.followTrajectorySequence(LeftAuto);

        if(tagOfInterest != null){

            if(tagOfInterest.id == POS_1_TAG_ID){


                Mec.followTrajectorySequence(Pos1Park);


            }else if(tagOfInterest.id == POS_2_TAG_ID){

                Mec.followTrajectorySequence(Pos2Park);
            }
            else if(tagOfInterest.id == POS_3_TAG_ID){

                //no need to move
            }

        }else{
            //park anywhere, 1 in 3 chance?
            Mec.followTrajectorySequence(Pos1Park);
        }

    }
}
