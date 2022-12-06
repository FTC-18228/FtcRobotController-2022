package org.firstinspires.ftc.teamcode.botbuilders.drive.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants;
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
public class NatBotBuildersPlowRightAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AprilTagDetection tagOfInterest = null;

    Pose2d StartPose = new Pose2d(35, -72, Math.toRadians(90));

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
        //region RightAutoTrajSequence
        //Drive.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
        Mec.setPoseEstimate(StartPose);
        //working
        TrajectorySequence NewRightAuto = Mec.trajectorySequenceBuilder(StartPose)
                .setConstraints(
                        BotBuildersMecanumDrive.getVelocityConstraint(
                                40, BotBuildersDriveConstants.MAX_ANG_VEL, BotBuildersDriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(
                                           BotBuildersDriveConstants.MAX_ACCEL)
                )
                .forward(12)
                .turn(Math.toRadians(90))
                .lineToSplineHeading(new Pose2d(35, -5, Math.toRadians(180)))
                .waitSeconds(0.1)
                .lineToSplineHeading(new Pose2d(35, -15, Math.toRadians(135)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    Mec.VertSlideToPos(3, 0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                    Mec.SlideServoOut();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                    Mec.ClawRelease();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                    Mec.ClawGrip();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                    Mec.SlideServoIn();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()-> {
                    Mec.VertSlideToPos(0, 0.7);
                })
                .forward(5)
                .lineToSplineHeading(new Pose2d(56, -12, Math.toRadians(0)))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    Mec.VertSlideToPos(1, 0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    Mec.ClawGrip();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    Mec.VertSlideUp(0.7);
                })
                .lineToSplineHeading(new Pose2d(34, -12, Math.toRadians(135)))
                .UNSTABLE_addTemporalMarkerOffset(1, ()-> {
                    Mec.VertSlideToPos(3, 0.7);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                    Mec.SlideServoOut();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                    Mec.ClawRelease();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1.1, ()-> {
                    Mec.ClawGrip();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2, ()-> {
                    Mec.SlideServoIn();
                })
                .UNSTABLE_addTemporalMarkerOffset(1.3, ()-> {
                    Mec.VertSlideToPos(0, 0.7);
                })
                .build();


        TrajectorySequence Pos2Park = Mec.trajectorySequenceBuilder(NewRightAuto.end())
                .strafeRight(26)
                .build();

        TrajectorySequence Pos3Park = Mec.trajectorySequenceBuilder(NewRightAuto.end())
                .strafeRight(55)
                .build();

        waitForStart();

        //Move claw around
        Mec.RearArmMid(0.3);
        Mec.VertSlideToPos(1, 0.8);
        Mec.SlideServoPickUp();

        Mec.followTrajectorySequence(NewRightAuto);

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

        /* if(tagOfInterest != null){

            if(tagOfInterest.id == POS_1_TAG_ID){

                //No need to move


            }else if(tagOfInterest.id == POS_2_TAG_ID){

               // Mec.followTrajectorySequence(Pos2Park);
            }
            else if(tagOfInterest.id == POS_3_TAG_ID){

               // Mec.followTrajectorySequence(Pos3Park);
            }

        }else{
            //park anywhere, 1 in 3 chance?
            Mec.followTrajectorySequence(Pos1Park);
        }*/
    }
}
