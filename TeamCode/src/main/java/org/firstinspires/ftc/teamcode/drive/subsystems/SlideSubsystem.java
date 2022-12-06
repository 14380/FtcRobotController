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

        leftSlide.setTargetPosition(460);
        rightSlide.setTargetPosition(460);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);

    }

    public void ManualSlideUp(double speed){

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(rightSlide.getCurrentPosition() < 1500 && leftSlide.getCurrentPosition() < 1500){
            rightSlide.setPower(speed);
            leftSlide.setPower(speed);
        }else{
            rightSlide.setPower(0);
            leftSlide.setPower(0);
        }
    }

    public void ManualSlideDown(double speed){

        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(rightSlide.getCurrentPosition() > 0 && leftSlide.getCurrentPosition() > 0){
            rightSlide.setPower(speed * -1);
            leftSlide.setPower(speed * -1);
        }else{
            rightSlide.setPower(0);
            leftSlide.setPower(0);
        }

    }

    public void SlideToMidAuto2(){

        telemetry.addData("SLIDE", "MID AUTO");

        leftSlide.setTargetPosition(1150);
        rightSlide.setTargetPosition(1150);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);

    }

    public void SlideToMidAuto() {
        telemetry.addData("SLIDE", "MID AUTO");

        leftSlide.setTargetPosition(750);
        rightSlide.setTargetPosition(750);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);
    }

    public void SlideToMid1Stack()
    {
        telemetry.addData("SLIDE", "MIDSTACK1");

        leftSlide.setTargetPosition(800);
        rightSlide.setTargetPosition(800);

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

        return leftSlide.getCurrentPosition() >  400 && rightSlide.getCurrentPosition() > 400 && leftSlide.getCurrentPosition() <  520 && rightSlide.getCurrentPosition() < 520;
    }

    public boolean IsSlideAtMidAuto(){

        return leftSlide.getCurrentPosition() >  450 && rightSlide.getCurrentPosition() > 450 && leftSlide.getCurrentPosition() <  800 && rightSlide.getCurrentPosition() < 800;
    }

    public boolean IsSlideAtMidAuto2(){

        return leftSlide.getCurrentPosition() >  950 && rightSlide.getCurrentPosition() > 950 && leftSlide.getCurrentPosition() <  1200 && rightSlide.getCurrentPosition() < 1200;
    }

    public boolean IsSlideAtMidStack1(){
        return leftSlide.getCurrentPosition() >  700 && rightSlide.getCurrentPosition() > 700 && leftSlide.getCurrentPosition() <  900 && rightSlide.getCurrentPosition() < 900;

    }

    public boolean IsAtGrasp(){

        return leftSlide.getCurrentPosition() >  390 && rightSlide.getCurrentPosition() > 390 && leftSlide.getCurrentPosition() <  480 && rightSlide.getCurrentPosition() < 480;

    }

}
