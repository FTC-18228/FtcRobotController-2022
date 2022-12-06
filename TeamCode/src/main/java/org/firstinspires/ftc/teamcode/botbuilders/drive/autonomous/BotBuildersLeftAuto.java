package org.firstinspires.ftc.teamcode.botbuilders.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
                .strafeRight(25)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    Mec.VertSlideToPos(3, 0.7);
                })
                .forward(15)
                .turn(Math.toRadians(-25))
                .UNSTABLE_addTemporalMarkerOffset(3.5, ()-> {
                    Mec.ClawRelease();
                    sleep(250);
                    Mec.VertSlideToPos(0,0.7);
                })
                .waitSeconds(3.5) //wait for the slides to catch up
                .turn(Math.toRadians(25))
                .back(2)
                .forward(32)
                .turn(Math.toRadians(90))
                .forward(36)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                    Mec.SlideServoAutoPickUp();
                })
                .forward(4)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                    Mec.ClawGrip();
                    sleep(350);
                    Mec.VertSlideToAutoPickupPos();
                })
                .forward(1)
                .waitSeconds(2)
                .back(20)
                .turn(Math.toRadians(40))
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                    sleep(250);
                    Mec.ClawRelease();
                })
                .waitSeconds(1.5)
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                    Mec.ClawGrip();
                })
                .turn(Math.toRadians(-35))
                .UNSTABLE_addTemporalMarkerOffset(0.1, ()-> {
                    Mec.SlideServoOut();
                    Mec.RotateClaw(0);
                    sleep(1000);
                    Mec.SlideServoToPos(0);
                    sleep(250);
                    Mec.VertSlideToPos(0, 0.7);
                })
                .waitSeconds(2)
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
        Mec.RearArmMid(0.3);

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
