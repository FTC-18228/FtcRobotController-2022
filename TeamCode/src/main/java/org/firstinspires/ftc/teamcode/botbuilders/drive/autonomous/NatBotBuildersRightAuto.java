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
public class NatBotBuildersRightAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    AprilTagDetection tagOfInterest = null;

    private int POS_1_TAG_ID = 1;
    private int POS_2_TAG_ID = 2;
    private int POS_3_TAG_ID = 3;

    Pose2d DeliveryPose2d = new Pose2d(12,-12, Math.toRadians(45));
    Pose2d StartPose2d = new Pose2d(34, -63, 1.5708);

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
        Mec.setPoseEstimate(new Pose2d(0, 0));
        //working
        TrajectorySequence NewRightAuto = Mec.trajectorySequenceBuilder(StartPose2d)
                .lineToSplineHeading(new Pose2d(12,-63, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(12,-35, Math.toRadians(130)))
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(12,-12, Math.toRadians(90)))
                .turn(Math.toRadians(-90))
                .lineToSplineHeading(new Pose2d(56,-12, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(0)))
                .lineToSplineHeading(DeliveryPose2d)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-45))
                .lineToSplineHeading(new Pose2d(56,-12, Math.toRadians(0)))
                .waitSeconds(0.5)
                .lineToSplineHeading(new Pose2d(24,-12, Math.toRadians(0)))
                .lineToSplineHeading(DeliveryPose2d)
                .waitSeconds(0.5)
                .turn(Math.toRadians(-45))
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

                Mec.followTrajectorySequence(Pos2Park);
            }
            else if(tagOfInterest.id == POS_3_TAG_ID){

                Mec.followTrajectorySequence(Pos3Park);
            }

        }else{
            //park anywhere, 1 in 3 chance?
            Mec.followTrajectorySequence(Pos1Park);
        }*/
    }
}
