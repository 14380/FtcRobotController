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
public class ArmReset extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        BotBuildersMecanumDrive mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        mecDrive.ReAlignIMU();

        GamepadEx gp1 = new GamepadEx(gamepad1);


        mecDrive.midPointIntake();

        waitForStart();
        while (!isStopRequested()) { // while robot is running and stop button is not pressed

            Pose2d poseEstimate = mecDrive.getPoseEstimate();

            gp1.readButtons();


            if (gp1.isDown(GamepadKeys.Button.DPAD_UP)) {
                mecDrive.UncontrolledUp(0.4);
            }
            else if (gp1.isDown(GamepadKeys.Button.DPAD_DOWN)){
                mecDrive.UncontrolledDown(0.2);
            }else{
                mecDrive.HoldArm();
            }

            if(gp1.isDown(GamepadKeys.Button.B)){
                mecDrive.ResetArmEncoders();
                telemetry.addData("Encoders RESET","");
                telemetry.update();
            }

        }

    }

}


