package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;

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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

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
    private DcMotorEx dr4bLeft;
    private DcMotorEx dr4bRight;

    private Servo rightServo, leftServo;
    private CRServo intakeServo;
    private List<Servo> servoList;
    private List<CRServo> CRServoList;
    private List<DcMotorEx> motors;

    private Servo clawArm;
    private Servo clawWrist;
    private Servo claw;

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;


    private BNO055IMU imu;
    private VoltageSensor batteryVoltageSensor;

    public static int SLIDE_MAX_HEIGHT = 2000;

    public static int MAX_ARM_POS = 1150;

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
        ReAlignIMU();

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        rightServo = hardwareMap.get(Servo.class, "rightServo");
        leftServo = hardwareMap.get(Servo.class, "leftServo");

        intakeServo = hardwareMap.get(CRServo.class, "IntakeServo");

        rightSlide = hardwareMap.get(DcMotorEx.class, "rightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "leftSlide");

        dr4bLeft = hardwareMap.get(DcMotorEx.class, "dr4vLeft");
        dr4bRight = hardwareMap.get(DcMotorEx.class, "dr4vRight");

        claw = hardwareMap.get(Servo.class, "claw");
        clawArm = hardwareMap.get(Servo.class, "clawArm");
        clawWrist = hardwareMap.get(Servo.class, "clawWrist");


        dr4bRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dr4bLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        servoList = Arrays.asList(rightServo, leftServo);
        CRServoList = Arrays.asList(intakeServo);
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


        //rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
       // rightFront.setDirection(DcMotorSimple.Direction.FORWARD );
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.REVERSE);
        intakeServo.setDirection(CRServo.Direction.REVERSE);

        //retractIntake();

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));

        trajectorySequenceRunner = new TrajectorySequenceRunner(follower, HEADING_PID);




    }


    public void ReAlignIMU(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        BNO055IMUUtil.remapAxes(imu, AxesOrder.YXZ, AxesSigns.PNP);

    }
    public void setServoPosition(double pos){
        leftServo.setPosition(pos);
        rightServo.setPosition(pos);
    }
    public void intakeOutReady(){


        leftServo.setPosition(0.89f);
        rightServo.setPosition(0.11);

    }

    public void intakeOutConePos(int coneHeight){
        //TODO: find the heights of the cone positions
        leftServo.setPosition(0.7f);
        rightServo.setPosition(0.3);
    }
    public void intakeDeliver(){
        leftServo.setPosition(0.4f);
        rightServo.setPosition(0.6f);
    }

    public void midPointIntake(){
        leftServo.setPosition(0.5);
        rightServo.setPosition(0.5);
    }

    public void CloseClaw(){
        claw.setPosition(0.25);
    }
    public void OpenClaw(){
        claw.setPosition(0);
    }

    ///Set ready for the claw to close
    public void ClawArmSet(){
        clawArm.setPosition(1);
    }

    //further away from the delivery area.
    public void ClawArmIntakeSet(){
        clawArm.setPosition(0.7);
    }

    //position of the arm for pickup pickup waiting
    public void ClawArmIntakePickUp(){clawArm.setPosition(0.55);};

    //position of the arm for delivery
    public void ClawArmDeliver1(){
        clawArm.setPosition(0.15);
    }

    public void ClawWristInPlace(){
        clawWrist.setPosition(1);
    }

    public boolean IsSlideIn(){

        if(rightSlide.getCurrentPosition() < 100){
            return true;
        }
        return false;
    }

    public void SlideIn(double speed){

        if(rightSlide.getCurrentPosition() > 0 && leftSlide.getCurrentPosition() > 0){
            rightSlide.setPower(speed * -1);
            leftSlide.setPower(speed * -1);
        }else{
            rightSlide.setPower(0);
            leftSlide.setPower(0);
        }

    }

    public void SlideOut(double speed){

        if(rightSlide.getCurrentPosition() < SLIDE_MAX_HEIGHT && leftSlide.getCurrentPosition() < SLIDE_MAX_HEIGHT){
            rightSlide.setPower(speed);
            leftSlide.setPower(speed);
        }else{
            rightSlide.setPower(0);
            leftSlide.setPower(0);
        }
    }

    public boolean isLiftArmUp(){
        if(dr4bRight.getCurrentPosition() > 600 || dr4bLeft.getCurrentPosition() > 600){
            return true;
        }
        return false;
    }

    public boolean isLiftArmDown(){

        if(dr4bRight.getCurrentPosition() < 100 || dr4bLeft.getCurrentPosition() < 100){
            return true;
        }
        return false;
    }

    public void ClawWristTurned(){
        //only turn this if the lift arm is up past a certain point
        if(dr4bRight.getCurrentPosition() > 600) {
            clawWrist.setPosition(0);
        }
    }

    public void ClawWristTurnedSide(){
        //only turn this if the lift arm is up past a certain point
        if(dr4bRight.getCurrentPosition() > 600) {
            clawWrist.setPosition(0.4);
        }
    }

    public void DecrementIntakePosition(double amount){
        leftServo.setPosition(leftServo.getPosition() + amount);
        rightServo.setPosition(rightServo.getPosition() - amount);
    }

    public void IncrementIntakePosition(double amount){
        leftServo.setPosition(leftServo.getPosition() - amount);
        rightServo.setPosition(rightServo.getPosition() + amount);
    }
    public void turnOnIntake(double magnitude){

        intakeServo.setPower(magnitude);

    }

    public void DropDr4B(double speed){

        dr4bLeft.setTargetPosition(0);
        dr4bRight.setTargetPosition(0);

        dr4bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr4bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dr4bRight.setPower(speed);
        dr4bLeft.setPower(speed);
    }

    public void LiftDr4BMid(double speed){
        dr4bLeft.setTargetPosition(MAX_ARM_POS - 150);
        dr4bRight.setTargetPosition(MAX_ARM_POS - 150);

        dr4bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr4bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dr4bRight.setPower(speed);
        dr4bLeft.setPower(speed);
    }

    public void LiftDr4BLow(double speed){
        dr4bLeft.setTargetPosition(MAX_ARM_POS - 220);
        dr4bRight.setTargetPosition(MAX_ARM_POS - 220);

        dr4bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr4bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dr4bRight.setPower(speed);
        dr4bLeft.setPower(speed);
    }

    public void LiftDr4B(double speed){

        dr4bLeft.setTargetPosition(MAX_ARM_POS);
        dr4bRight.setTargetPosition(MAX_ARM_POS);

        dr4bLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dr4bRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        dr4bRight.setPower(speed);
        dr4bLeft.setPower(speed);
    }

    public void DumpData(Telemetry telemetry){
        telemetry.addData("Left 4B", dr4bLeft.getCurrentPosition());
        telemetry.addData("Right 4B", dr4bRight.getCurrentPosition());
        telemetry.addData("Left Slide", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide", rightSlide.getCurrentPosition());

        telemetry.addData("Right Front", rightFront.getCurrentPosition());
        telemetry.addData("Left Front", leftFront.getCurrentPosition());
        telemetry.addData("Right Rear", rightRear.getCurrentPosition());
        telemetry.addData("Left Rear", leftRear.getCurrentPosition());

        telemetry.update();
    }


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
        // TODO: This must be changed to match your configuration
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // The positive x axis points toward the USB port(s)
        //
        // Adjust the axis rotation rate as necessary
        // Rotate about the z axis is the default assuming your REV Hub/Control Hub is laying
        // flat on a surface

        return (double) imu.getAngularVelocity().zRotationRate;
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

    public int getRightPos(){
        return rightSlide.getCurrentPosition();
    }

    public int getLeftPos(){
        return leftSlide.getCurrentPosition();
    }




}
