package org.firstinspires.ftc.teamcode.botbuilders.drive;

import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.kA;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.botbuilders.drive.BotBuildersDriveConstants.kV;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.demo.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.trajectorysequence.TrajectorySequenceRunner;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for REV hardware.
 */
@Config
public class BotBuildersMecanumDrive extends MecanumDrive {
    public static PIDCoefficients TRANSLATIONAL_PID = new PIDCoefficients(9, 0, 0.01);
    public static PIDCoefficients HEADING_PID = new PIDCoefficients(8, 0, 0.08);

    public static double LATERAL_MULTIPLIER = 1;

    public static double VX_WEIGHT = 0.05;
    public static double VY_WEIGHT = 0.05;
    public static double OMEGA_WEIGHT = 0.05;

    private TrajectorySequenceRunner trajectorySequenceRunner;

    private static final TrajectoryVelocityConstraint VEL_CONSTRAINT = getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH);
    private static final TrajectoryAccelerationConstraint ACCEL_CONSTRAINT = getAccelerationConstraint(MAX_ACCEL);

    private TrajectoryFollower follower;

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;
    private OpenCvCamera camera;
    private AprilTagDetectionPipeline AprilTagPipeline;

    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    //servos for intake
    private Servo leftRearArm;
    private Servo rightRearArm;
    private CRServo intakeServo;

    //servo for claw
    private Servo clawServo;

    private Servo clawServo2;

    //servo for slide arm
    private Servo slideServo;

    //motors for vert slides

    private DcMotor leftVertSlide;
    private DcMotor rightVertSlide;

    private DcMotor leftCampSlide;
    private DcMotor rightCampSlide;

    private int CAMP_SLIDE_MAX = 1800;
    private int VERT_SLIDE_MAX = 3100;

    public BotBuildersMecanumDrive(HardwareMap hardwareMap) {
        super(kV, kA, kStatic, TRACK_WIDTH, TRACK_WIDTH, LATERAL_MULTIPLIER);

        follower = new HolonomicPIDVAFollower(TRANSLATIONAL_PID, TRANSLATIONAL_PID, HEADING_PID,
                new Pose2d(0.5, 0.5, Math.toRadians(5.0)), 0.5);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: If the hub containing the IMU you are using is mounted so that the "REV" logo does
        // not face up, remap the IMU axes so that the z-axis points upward (normal to the floor.)
        //
        //             | +Z axis
        //             |
        //             |
        //             |
        //      _______|_____________     +Y axis
        //     /       |_____________/|__________
        //    /   REV / EXPANSION   //
        //   /       / HUB         //
        //  /_______/_____________//
        // |_______/_____________|/
        //        /
        //       / +X axis
        //
        // This diagram is derived from the axes in section 3.4 https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
        // and the placement of the dot/orientation from https://docs.revrobotics.com/rev-control-system/control-system-overview/dimensions#imu-location
        //
        // For example, if +Y in this diagram faces downwards, you would use AxisDirection.NEG_Y.
        // BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_Y);

        leftRearArm = hardwareMap.get(Servo.class, "leftRearArm");
        rightRearArm = hardwareMap.get(Servo.class, "rightRearArm");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        slideServo = hardwareMap.get(Servo.class, "slideServo");

        clawServo = hardwareMap.get(Servo.class, "clawServo");

        clawServo2 = hardwareMap.get(Servo.class, "clawServo2");

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftCampSlide = hardwareMap.get(DcMotor.class, "leftCampSlide");
        rightCampSlide = hardwareMap.get(DcMotor.class, "rightCampSlide");

        leftCampSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightCampSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightCampSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftVertSlide = hardwareMap.get(DcMotor.class, "leftVertSlide");
        rightVertSlide = hardwareMap.get(DcMotor.class, "rightVertSlide");


        leftVertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVertSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        if (RUN_USING_ENCODER) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        //need to reverse one of the arm servos;
        leftRearArm.setDirection(Servo.Direction.REVERSE);



        //18228 REAL
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //TESTBOT
       // rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        //rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);
    }
    //TODO: Create methods ResetSlideEncoder, ReAlignIMU, Linear Slide functions, AprilTagDetection
    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose) {
        return new TrajectoryBuilder(startPose, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, boolean reversed) {
        return new TrajectoryBuilder(startPose, reversed, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectoryBuilder trajectoryBuilder(Pose2d startPose, double startHeading) {
        return new TrajectoryBuilder(startPose, startHeading, VEL_CONSTRAINT, ACCEL_CONSTRAINT);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d startPose) {
        return new TrajectorySequenceBuilder(
                startPose,
                VEL_CONSTRAINT, ACCEL_CONSTRAINT,
                MAX_ANG_VEL, MAX_ANG_ACCEL
        );
    }

    public void turnAsync(double angle) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(getPoseEstimate())
                        .turn(angle)
                        .build()
        );
    }

    public void turn(double angle) {
        turnAsync(angle);
        waitForIdle();
    }

    public void followTrajectoryAsync(Trajectory trajectory) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(
                trajectorySequenceBuilder(trajectory.start())
                        .addTrajectory(trajectory)
                        .build()
        );
    }

    public void followTrajectory(Trajectory trajectory) {
        followTrajectoryAsync(trajectory);
        waitForIdle();
    }

    public void followTrajectorySequenceAsync(TrajectorySequence trajectorySequence) {
        trajectorySequenceRunner.followTrajectorySequenceAsync(trajectorySequence);
    }

    public void followTrajectorySequence(TrajectorySequence trajectorySequence) {
        followTrajectorySequenceAsync(trajectorySequence);
        waitForIdle();
    }

    public Pose2d getLastError() {
        return trajectorySequenceRunner.getLastPoseError();
    }

    public void update() {
        updatePoseEstimate();
        DriveSignal signal = trajectorySequenceRunner.update(getPoseEstimate(), getPoseVelocity());
        if (signal != null) setDriveSignal(signal);
    }

    public void waitForIdle() {
        while (!Thread.currentThread().isInterrupted() && isBusy())
            update();
    }

    public boolean isBusy() {
        return trajectorySequenceRunner.isBusy();
    }

    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public void setWeightedDrivePower(Pose2d drivePower) {
        Pose2d vel = drivePower;

        if (Math.abs(drivePower.getX()) + Math.abs(drivePower.getY())
                + Math.abs(drivePower.getHeading()) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(drivePower.getX())
                    + VY_WEIGHT * Math.abs(drivePower.getY())
                    + OMEGA_WEIGHT * Math.abs(drivePower.getHeading());

            vel = new Pose2d(
                    VX_WEIGHT * drivePower.getX(),
                    VY_WEIGHT * drivePower.getY(),
                    OMEGA_WEIGHT * drivePower.getHeading()
            ).div(denom);
        }

        setDrivePower(vel);
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelVelocities.add(encoderTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }

    @Override
    public Double getExternalHeadingVelocity() {
        // To work around an SDK bug, use -zRotationRate in place of xRotationRate
        // and -xRotationRate in place of zRotationRate (yRotationRate behaves as 
        // expected). This bug does NOT affect orientation. 
        //
        // See https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/251 for details.
        return (double) -imu.getAngularVelocity().xRotationRate;
    }

    public static TrajectoryVelocityConstraint getVelocityConstraint(double maxVel, double maxAngularVel, double trackWidth) {
        return new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(maxAngularVel),
                new MecanumVelocityConstraint(maxVel, trackWidth)
        ));
    }

    public static TrajectoryAccelerationConstraint getAccelerationConstraint(double maxAccel) {
        return new ProfileAccelerationConstraint(maxAccel);
    }

    // START robot functions

    public void RearArmIn(){
        leftRearArm.setPosition(0.55);
        rightRearArm.setPosition(0.55);
    }



    public void RearArmMid(){
        leftRearArm.setPosition(0.6);
        rightRearArm.setPosition(0.6);
    }

    public void RearArmOut(){
        leftRearArm.setPosition(1);
        rightRearArm.setPosition(1);
    }

    public void RearArmDownIncr(){
        leftRearArm.setPosition(leftRearArm.getPosition() + 0.01);
        rightRearArm.setPosition(leftRearArm.getPosition() + 0.01);
    }

    public void RearArmUpIncr(){
        leftRearArm.setPosition(leftRearArm.getPosition() - 0.01);
        rightRearArm.setPosition(leftRearArm.getPosition() - 0.01);
    }

    //set the intake to a given speed
    public void IntakeSpeed(double speed){
        if(speed != 0) {
            intakeServo.setPower(speed);
        }
        else{
            intakeServo.setPower(0);
        }
    }

    //Grips the cone
    public void ClawGrip(){
        clawServo.setPosition(0.5);
        clawServo2.setPosition(0.3);
    }

    //Releases the claw - drop off the cone
    public void ClawRelease(){
        clawServo.setPosition(0.8);
        clawServo2.setPosition(0);
    }

    //Brings the linear slide arm servo into position - ready to grip the cone
    public void SlideServoOut(){
        slideServo.setPosition(1);
    }

    public void WriteData(Telemetry tele){
        tele.addData("SlideArm Pos", slideServo.getPosition());
        tele.addData("Left Front Wheel", leftFront.getCurrentPosition());
        tele.addData("Right Front Wheel", rightFront.getCurrentPosition());
        tele.addData("Left Rear Wheel", leftRear.getCurrentPosition());
        tele.addData("Right Rear Wheel", rightRear.getCurrentPosition());

        tele.addData("Left Camp Slide", leftCampSlide.getCurrentPosition());
        tele.addData("Right Camp Slide", rightCampSlide.getCurrentPosition());
        tele.addData("Left Vert Slide", leftVertSlide.getCurrentPosition());
        tele.addData("Right Vert Slide", rightVertSlide.getCurrentPosition());
        tele.addData("ClawGrip", clawServo.getPosition());

        tele.update();
    }

    public void ResetCampSlideEncoders(){
        leftCampSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightCampSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ResetVertSlideEncoders(){
        leftVertSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVertSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //moves the linear slide arm servo Ready to pick up cone
    public void SlideServoIn(){
        slideServo.setPosition(0.3);
    }

    public void SlideServoPickUp(){
        slideServo.setPosition(0.15);
    }

    public void ReAlignIMU(){
        BNO055IMU.Parameters params = new BNO055IMU.Parameters();
        params.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(params);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.YXZ, AxesSigns.PNP);
    }


    public void CampSlideOut(double speed){

        leftCampSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightCampSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftCampSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCampSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(rightCampSlide.getCurrentPosition() < CAMP_SLIDE_MAX && leftCampSlide.getCurrentPosition() < CAMP_SLIDE_MAX){
            leftCampSlide.setPower(speed);
            rightCampSlide.setPower(speed);
        }else{
            leftCampSlide.setPower(0);
            rightCampSlide.setPower(0);
        }
    }

    public void CampSlideIn(double speed){

        leftCampSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightCampSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        leftCampSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightCampSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(rightCampSlide.getCurrentPosition() < 0 && leftCampSlide.getCurrentPosition() < 0){
            leftCampSlide.setPower(speed);
            rightCampSlide.setPower(speed);
        }else{
            leftCampSlide.setPower(0);
            rightCampSlide.setPower(0);
        }
    }
    //this function will return a delay amount, based on how far the camp slide is from the base
    //these numbers are experimental
    public void CampSlideDelay(LinearOpMode mode){

        if(leftCampSlide.getCurrentPosition() < 10 && rightCampSlide.getCurrentPosition() < 10) {
            //no need for delay
            return;
        }else if(leftCampSlide.getCurrentPosition() < 100 && rightCampSlide.getCurrentPosition() < 100) {
            //just a small sleep
            mode.sleep(100);
        } else if(leftCampSlide.getCurrentPosition() < 1000 && rightCampSlide.getCurrentPosition() < 1000){
            mode.sleep(250);
        } else if(leftCampSlide.getCurrentPosition() < 2000 && rightCampSlide.getCurrentPosition() < 2000){
            mode.sleep(550);
        }else{
            mode.sleep(750);
        }
    }

    //Sets Camp Slides to given position and if <= 0 resets Camp Slides position
    public void CampSlideToPos(int pos, double speed){
        leftCampSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightCampSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(pos > 0){
            leftCampSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            rightCampSlide.setDirection(DcMotorSimple.Direction.FORWARD);

            leftCampSlide.setTargetPosition(pos);
            rightCampSlide.setTargetPosition(pos);

            leftCampSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightCampSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftCampSlide.setPower(speed);
            rightCampSlide.setPower(speed);

        }
        if(pos <= 0){
            leftCampSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            rightCampSlide.setDirection(DcMotorSimple.Direction.REVERSE);

            leftCampSlide.setTargetPosition(0);
            rightCampSlide.setTargetPosition(0);

            leftCampSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightCampSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftCampSlide.setPower(speed);
            rightCampSlide.setPower(speed);
        }
    }

    public void VertSlideDown(double speed){
        rightVertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        leftVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        rightVertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftVertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(rightVertSlide.getCurrentPosition() < 0 && leftVertSlide.getCurrentPosition() < 0){
            rightVertSlide.setPower(speed);
            leftVertSlide.setPower(speed);
        }else{
            rightVertSlide.setPower(0);
            leftVertSlide.setPower(0);
        }
    }

    public void IntakeSlideUp(){
        rightVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        rightVertSlide.setTargetPosition(200);
        leftVertSlide.setTargetPosition(200);

        rightVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightVertSlide.setPower(0.7);
        leftVertSlide.setPower(0.7);

    }

    public void IntakeSlideDown(){

        rightVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        rightVertSlide.setTargetPosition(0);
        leftVertSlide.setTargetPosition(0);

        rightVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightVertSlide.setPower(0.7);
        leftVertSlide.setPower(0.7);
    }

    public void VertSlideUp(double speed){

        rightVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftVertSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        rightVertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftVertSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(rightVertSlide.getCurrentPosition() < VERT_SLIDE_MAX && leftVertSlide.getCurrentPosition() < VERT_SLIDE_MAX){
            rightVertSlide.setPower(speed);
            leftVertSlide.setPower(speed);
        }else{
            rightVertSlide.setPower(0);
            leftVertSlide.setPower(0);
        }
    }

    //Sets Linear Slides to given position and if <= 0 reset Linear Slides position
    public void VertSlideToPos(int pos, double speed){
        leftVertSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVertSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(pos == 3){
            leftVertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            rightVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);

            leftVertSlide.setTargetPosition(VERT_SLIDE_MAX);
            rightVertSlide.setTargetPosition(VERT_SLIDE_MAX);

            leftVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftVertSlide.setPower(speed);
            rightVertSlide.setPower(speed);

        }
        else if(pos == 2){
            leftVertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            rightVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);

            leftVertSlide.setTargetPosition(1800);
            rightVertSlide.setTargetPosition(1800);

            leftVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftVertSlide.setPower(speed);
            rightVertSlide.setPower(speed);

        }
        else if(pos <= 0){
            leftVertSlide.setDirection(DcMotorSimple.Direction.FORWARD);
            rightVertSlide.setDirection(DcMotorSimple.Direction.REVERSE);

            leftVertSlide.setTargetPosition(0);
            rightVertSlide.setTargetPosition(0);

            leftVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightVertSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftVertSlide.setPower(speed);
            rightVertSlide.setPower(speed);
        }
    }
}
