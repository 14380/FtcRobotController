package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideSubsystem extends SubsystemBase {


    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private final Telemetry telemetry;

    public SlideSubsystem(HardwareMap map, Telemetry tele)
    {
        leftSlide = map.get(DcMotorEx.class, "leftSlide");
        rightSlide = map.get(DcMotorEx.class, "rightSlide");
        telemetry = tele;
    }

    public void SlideToTop(){
        telemetry.addData("SLIDE", "Top");
        leftSlide.setTargetPosition(1500);
        rightSlide.setTargetPosition(1500);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.9);
        rightSlide.setPower(0.9);
    }

    public void SlideToBottom(){
        telemetry.addData("SLIDE", "Bottom");

        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);
    }

    public void SlideToMid(){
        telemetry.addData("SLIDE", "MID");

        leftSlide.setTargetPosition(500);
        rightSlide.setTargetPosition(500);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);

    }

    public void SlideToGrasp(){
        telemetry.addData("SLIDE", "GRASP");

        leftSlide.setTargetPosition(400);
        rightSlide.setTargetPosition(400);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);
    }

    public boolean IsSlideAtTop(){
        // Is at the top
        return leftSlide.getCurrentPosition() >  1400 && rightSlide.getCurrentPosition() > 1400;
    }

    public boolean IsSlideAtBottom(){
        // Is at the bottom
        return leftSlide.getCurrentPosition() < 100 && rightSlide.getCurrentPosition() < 100;
    }

    public boolean IsSlideAtMid(){
        // Is at the top
        return leftSlide.getCurrentPosition() >  460 && rightSlide.getCurrentPosition() > 460 && leftSlide.getCurrentPosition() <  520 && rightSlide.getCurrentPosition() < 520;
    }

    public boolean IsAtGrasp(){

        return leftSlide.getCurrentPosition() >  390 && rightSlide.getCurrentPosition() > 390 && leftSlide.getCurrentPosition() <  430 && rightSlide.getCurrentPosition() < 430;

    }

}
