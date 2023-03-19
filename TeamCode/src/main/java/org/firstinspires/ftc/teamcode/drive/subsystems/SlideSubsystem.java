package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry = tele;
    }

    public void SlideToTop(){
        telemetry.addData("SLIDE", "Top");
        //WP: These two numbers control the top height - larger is higher
        leftSlide.setTargetPosition(1575);
        rightSlide.setTargetPosition(1575);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.9);
        rightSlide.setPower(0.9);
    }


    public void SlideToTop2(){
        telemetry.addData("SLIDE", "Top");
        leftSlide.setTargetPosition(1650);
        rightSlide.setTargetPosition(1650);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.9);
        rightSlide.setPower(0.9);
    }

    public void SlideToLinkageOut(){
        telemetry.addData("SLIDE", "Out");
        leftSlide.setTargetPosition(1400);
        rightSlide.setTargetPosition(1400);

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

        //WP: Mid Auto second and third cone

        leftSlide.setTargetPosition(1200);
        rightSlide.setTargetPosition(1200);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);

    }

    public void SlideToMidAuto() {
        telemetry.addData("SLIDE", "MID AUTO");

        //WP: Change these for Mid Auto
        leftSlide.setTargetPosition(800);
        rightSlide.setTargetPosition(800);

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
        return leftSlide.getCurrentPosition() >  1500 && rightSlide.getCurrentPosition() > 1500;
    }

    public boolean IsSlideAtTop2(){
        // Is at the top
        return leftSlide.getCurrentPosition() >  1500 && rightSlide.getCurrentPosition() > 1500;
    }

    public boolean IsSlideToLinkageOut(){
        return leftSlide.getCurrentPosition() >  1300 && rightSlide.getCurrentPosition() > 1300;
    }
    public boolean IsSlideAtBottom(){
        // Is at the bottom
        return leftSlide.getCurrentPosition() < 100 && rightSlide.getCurrentPosition() < 100;
    }

    public boolean IsSlideAtMid(){

        return leftSlide.getCurrentPosition() >  400 && rightSlide.getCurrentPosition() > 400 && leftSlide.getCurrentPosition() <  520 && rightSlide.getCurrentPosition() < 520;
    }

    public boolean IsSlideAtGrasp(){
        return leftSlide.getCurrentPosition() >  300 && rightSlide.getCurrentPosition() > 300 && leftSlide.getCurrentPosition() <  520 && rightSlide.getCurrentPosition() < 520;

    }

    public boolean IsSlideAtMidAuto(){

        return leftSlide.getCurrentPosition() >  550 && rightSlide.getCurrentPosition() > 550 && leftSlide.getCurrentPosition() <  800 && rightSlide.getCurrentPosition() < 800;
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
