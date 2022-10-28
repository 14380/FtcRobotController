package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;

@Config
@TeleOp(group = "drive")
public class BBTeleOp extends LinearOpMode {
    private boolean clawGripped = false;
    private boolean wristInside = true;
    private boolean armOut = false;

    private boolean armIsReady = false;

    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        mecDrive.ReAlignIMU();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);


        ElapsedTime timer = new ElapsedTime();
        //used to keep track of the last time the up button was pressed.

        double velocity = 0.5;


        timer.startTime();

        mecDrive.midPointIntake();

        waitForStart();
        while (!isStopRequested()) { // while robot is running and stop button is not pressed

            Pose2d poseEstimate = mecDrive.getPoseEstimate();

            gp1.readButtons();
            gp2.readButtons();

            if (gamepad1.a && gamepad1.b) {
                telemetry.addData("Cleared IMU", "Done");
                mecDrive.ReAlignIMU();
            }
            //----------------------------------------------

            if(gp1.isDown(GamepadKeys.Button.DPAD_LEFT) ) {
                armIsReady = false;
                LiftArmToHigh(mecDrive, 1); //LOW POS
                //move the intake to the mid point to keep out of the way
                sleep(300);


            }else if(gp1.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
                armIsReady = false;
                LiftArmToHigh(mecDrive, 3); //HIGHEST POS
                //move the intake to the mid point to keep out of the way
                sleep(300);
                //give some time for the arm to go up.
                mecDrive.midPointIntake();
            }else if(gp1.wasJustReleased(GamepadKeys.Button.DPAD_RIGHT)){

                armIsReady = false;
                LiftArmToHigh(mecDrive, 2); //MID POS
                //move the intake to the mid point to keep out of the way
                sleep(300);
                //give some time for the arm to go up.
                mecDrive.midPointIntake();


            }

            //bring the arm down
            if(gamepad1.dpad_down){

                if(wristInside == false){
                    //the wrist is out, give us some time to bring it in

                    mecDrive.ClawArmIntakeSet();
                    sleep(200);

                    mecDrive.ClawWristInPlace();

                    sleep(500);
                    mecDrive.OpenClaw();


                }
                clawGripped = false;
                wristInside = true;
                armOut = false;
                armIsReady = false;

                mecDrive.DropDr4B(0.2);

            }

            //X picks up and drops the cone.
            if(gp1.wasJustPressed(GamepadKeys.Button.X)) {
                clawGripped = !clawGripped;
                if(clawGripped == false && mecDrive.isLiftArmDown()){
                    //if the arm is down, then we are a little distance back
                    //to ensure that we don't contact the arm
                    //so we need to slide forward first before gripping.
                    //setting the above variable clawGripped will cause the
                    //claw to take the action we need.
                    mecDrive.ClawArmSet();


                }
            }

            //if the lift arm is high, and the Y button is pressed
            //we will move the wrist to the front of the robot
            //default is to the right
            if(gp1.wasJustPressed(GamepadKeys.Button.Y) && mecDrive.isLiftArmUp()){

                if(mecDrive.isLiftArmUp()){
                    //we should also take the claw out
                    mecDrive.ClawWristTurned();
                    sleep(100);
                    mecDrive.ClawArmDeliver1();
                    wristInside = false;
                    armOut = true;
                }
            }


            if(gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)){
                mecDrive.turnOnIntake(-1);
                sleep(350);
                mecDrive.intakeOutReady();
                sleep(100);
                armIsReady = true;
            }

            if(gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                mecDrive.intakeDeliver();
                armIsReady = false;
            }

            if(gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5 || gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
                mecDrive.SlideOut(0.5);
            }else if(gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 || gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5){
                mecDrive.SlideIn(0.5);

            }else{
                mecDrive.SlideOut(0);
            }

            if(clawGripped){
               // telemetry.addData("Claw", "Trigger Close");
                mecDrive.CloseClaw();

            }else{
                //telemetry.addData("Claw", "Trigger Open");
                mecDrive.OpenClaw();
            }
            telemetry.addData("armIsReady", armIsReady);
            telemetry.update();
            if(armIsReady) {
                mecDrive.ClawArmIntakeSet();
            }else {
                if (armOut) {
                    mecDrive.ClawArmDeliver1();
                } else {

                        mecDrive.ClawArmIntakePickUp();

                }
            }
            //driver 2 - fine movements for the back arm
            if(gp2.wasJustReleased(GamepadKeys.Button.DPAD_UP)){
                //raise the back arm
                mecDrive.DecrementIntakePosition(0.01);
            }else if(gp2.wasJustReleased(GamepadKeys.Button.DPAD_DOWN)){
                //lower the back arm
                mecDrive.IncrementIntakePosition(0.01);
            }

           mecDrive.DumpData(telemetry);

            //A and B control the intake mech
            if(gamepad1.b || gamepad2.b){
                mecDrive.turnOnIntake(1.f);
            }
            else  if(gamepad1.a || gamepad2.a){
                mecDrive.turnOnIntake(-1.f);
            }else{
                mecDrive.turnOnIntake(0.f);
            }

            //-----------------------------------------------
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * velocity,
                    -gamepad1.left_stick_x * velocity

            ).rotated(-poseEstimate.getHeading());

            if (gp1.isDown(GamepadKeys.Button.X)) {
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

           // telemetry.addData("Wrist In", wristInside);
           // telemetry.addData("Claw Gripped", clawGripped);
           // telemetry.addData("Arm Out", armOut);
           // telemetry.update();
        }


    }

    private void LiftArmToHigh(BotBuildersMecanumDrive mecDrive, int pos) {

        if(pos == 3) {
            mecDrive.LiftDr4B(0.6);
        }else if(pos == 2){
            mecDrive.LiftDr4BMid(0.6);
        }
        else if(pos == 1){
            mecDrive.LiftDr4BLow(0.6);
        }
        if(pos > 1) {
            if (mecDrive.isLiftArmUp()) {
                wristInside = false;
                sleep(100);
                armOut = true;

                mecDrive.ClawWristTurnedSide();  //ClawWristTurned();
                sleep(500);
                mecDrive.ClawArmDeliver1();

                clawGripped = true;
            }
        }
    }

}


