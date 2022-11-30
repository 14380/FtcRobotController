package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;

public class DriveSubsystem extends SubsystemBase {

    BotBuildersMecanumDrive driveBase;
    GamepadEx gp1;
    Telemetry telemetry;

    public DriveSubsystem(BotBuildersMecanumDrive drive, GamepadEx gamepad, Telemetry tele) {

        gp1 = gamepad;
        driveBase = drive;
        telemetry = tele;
    }

    public void Realign(){
        telemetry.addData("IMU", "RESET");
        driveBase.ReAlignIMU();
    }

    public void drive() {
        //telemetry.addData("DRIVE","Driving");
        Pose2d poseEstimate = driveBase.getPoseEstimate();
        double velocity = 0.5;

        Vector2d input = new Vector2d(
                -gp1.gamepad.left_stick_y * 0.75,
                -gp1.gamepad.left_stick_x * 0.75

        ).rotated(-poseEstimate.getHeading());


        Pose2d vel = new Pose2d(
                input.getX(),
                input.getY(),
                -gp1.gamepad.right_stick_x * velocity
        );

        driveBase.DumpData(telemetry);
        driveBase.setWeightedDrivePower(vel);
        driveBase.update();

    }
}
