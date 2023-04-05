package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {

   // private final Servo turretMotor;
    private final Telemetry telemetry;
    private final DcMotorEx turretEnc;

    public TurretSubsystem(HardwareMap map, Telemetry tele)
    {
      //  turretMotor = map.get(Servo.class, "turretServo");
        turretEnc = map.get(DcMotorEx.class, "turretEnc");
        telemetry = tele;
    }

    public void RotateToFront()
    {
        //need to make sure we rotate the correct way
        telemetry.addData("ROTATE", "FRONT");

        turretEnc.setTargetPosition(0);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.6);

    }

    public void RotateToFrontAuto()
    {
        //need to make sure we rotate the correct way
        telemetry.addData("ROTATE", "FRONT");

        turretEnc.setTargetPosition(-5);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.6);

    }

    public void RotateToFrontAutoLeft()
    {
        //need to make sure we rotate the correct way
        telemetry.addData("ROTATE", "FRONT");

        turretEnc.setTargetPosition(20);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.6);

    }


    public boolean IsAtFrontAutoLeft(){
        return    Math.abs(turretEnc.getCurrentPosition()) < 30;
    }

    public void RotateToFrontFast()
    {
        //need to make sure we rotate the correct way
        telemetry.addData("ROTATE", "FRONT");

        turretEnc.setTargetPosition(100);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.95);

    }

    public void RotateToFrontFastLeft()
    {
        //need to make sure we rotate the correct way
        telemetry.addData("ROTATE", "FRONT");

        turretEnc.setTargetPosition(-100);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.95);

    }

    public void RotateToRear(){

        telemetry.addData("ROTATE", "REAR");
       // turretMotor.setPosition(0.39);
        turretEnc.setTargetPosition(-960);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.8);
    }

    public void RotateLeft(){

        telemetry.addData("ROTATE", "LEFT");

        turretEnc.setTargetPosition(800);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.8);

    }



    public boolean IsAtLeft(){

        return Math.abs(turretEnc.getCurrentPosition()) > 750 && Math.abs(turretEnc.getCurrentPosition()) < 840;
    }




    public boolean IsAtAutoRight(){
        return true;//turretMotor.getPosition() > 0.25 && turretMotor.getPosition() < 0.4;
    }

    public void RotateAutoLeftClose(){

        telemetry.addData("ROTATE", "LEFT");

        turretEnc.setTargetPosition(838);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.5);

    }

    public boolean IsAtAutoLeftClose(){

        return Math.abs(turretEnc.getCurrentPosition()) > 830 && Math.abs(turretEnc.getCurrentPosition()) < 845;
    }

    public void RotatePosition(int position){

        telemetry.addData("ROTATE", "LEFT");

        turretEnc.setTargetPosition(position);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.5);

    }

    public boolean IsAtPosition(int lowpos, int highpos){

        return Math.abs(turretEnc.getCurrentPosition()) > lowpos && Math.abs(turretEnc.getCurrentPosition()) < highpos;
    }

    public void RotateAutoLeftCloseFast(){

        telemetry.addData("ROTATE", "LEFT");

        turretEnc.setTargetPosition(500);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.9);

    }
    public void RotateAutoRightClose(){

        turretEnc.setTargetPosition(-790);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.6);
    }

    public boolean IsAtAutoRightClose(){

        return Math.abs(turretEnc.getCurrentPosition()) > 775;
    }

    public void RotateAutoRightCloseFast(){

        turretEnc.setTargetPosition(-750);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(1);
    }

    public boolean IsAtAutoRightCloseFast(){

        return Math.abs(turretEnc.getCurrentPosition()) > 720;
    }

    public void ManualLeft(double speed){
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //if(turretEnc.getCurrentPosition() < 1000){
            turretEnc.setPower(speed);

        //}else{
        //    turretEnc.setPower(0);

        //}
    }

    public void ManualRight(double speed){
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //if(turretEnc.getCurrentPosition() < - 1000){
            turretEnc.setPower(-1 * speed);

        //}else{
        //    turretEnc.setPower(0);

        //}
    }

    public void RotateRight()
    {
        telemetry.addData("ROTATE", "RIGHT");


        turretEnc.setTargetPosition(-790);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.8);

    }

    public boolean IsAtRight(){

        return turretEnc.getCurrentPosition() < 0 && Math.abs(turretEnc.getCurrentPosition()) > 700 && Math.abs(turretEnc.getCurrentPosition()) < 950;
    }


    public boolean IsAtRear(){

        return Math.abs(turretEnc.getCurrentPosition()) > 900 && Math.abs(turretEnc.getCurrentPosition()) < 1100;
    }


    public boolean IsAtAutoLeftCloseFast(){

        return Math.abs(turretEnc.getCurrentPosition()) > 480 && Math.abs(turretEnc.getCurrentPosition()) < 750;
    }



    public boolean IsAtFront(){

        return    Math.abs(turretEnc.getCurrentPosition()) < 20;

    }


    public boolean IsAtFrontFast(){

        return    Math.abs(turretEnc.getCurrentPosition()) < 100;

    }


}
