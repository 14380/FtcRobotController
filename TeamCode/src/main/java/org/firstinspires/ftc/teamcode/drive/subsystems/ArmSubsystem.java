package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends SubsystemBase {

    private DcMotor armMotor;

    private final Telemetry telemetry;



    public ArmSubsystem(HardwareMap map, Telemetry tele)
    {

        telemetry = tele;

        armMotor = map.get(DcMotor.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void Top(){
        telemetry.addData("ARM", "TOP");
        armMotor.setTargetPosition(700);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.8);
    }

    public void TopAuto(){
        telemetry.addData("ARM", "TOP AUTO");
        telemetry.addData("ARM", "TOP");
        armMotor.setTargetPosition(850);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.9);
    }

    public void SuperTopAuto(){
        armMotor.setTargetPosition(950);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(1);
    }


    public void Mid(){
        armMotor.setTargetPosition(400);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.8);

        telemetry.addData("ARM", "MID");
    }

    public void MidStack5(int position){

        armMotor.setTargetPosition(position);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.8);

    }

    //the stack
    public void MidStack1(){


        telemetry.addData("ARM", "MID Stack 1");
    }

    public void MidStack2(){

        telemetry.addData("ARM", "MID Stack 2");
    }

    public boolean isAtTop(){
        return  armMotor.getCurrentPosition()  > 700;
    }

    public boolean isAtTopAuto(){
        return  armMotor.getCurrentPosition()  > 750;
    }

    public boolean isAtMid()
    {
        // Return when the mid point is located

        return armMotor.getCurrentPosition()  > 350 && armMotor.getCurrentPosition() < 450;
    }

    public boolean isAtMid5(int position)
    {
        // Return when the mid point is located

        return armMotor.getCurrentPosition()  > position - 100 && armMotor.getCurrentPosition() < position + 100;
    }

    public boolean isAtMidStack1()
    {
        // Return when the mid point is located

        return true; //leftServo.getPosition() > 0.3 && rightServo.getPosition() > 0.3 && rightServo.getPosition() < 0.45 && leftServo.getPosition() < 0.45;
    }

    public boolean isAtMidStack2()
    {
        // Return when the mid point is located

        return true; //leftServo.getPosition() > 0.3 && rightServo.getPosition() > 0.3 && rightServo.getPosition() < 0.5 && leftServo.getPosition() < 0.5;
    }

    public boolean isAtCone(){
        // Return true when the arm is at the bottom.
        return true;//armMotor.getCurrentPosition() > -10 && armMotor.getCurrentPosition() < 10;
    }


    public void ReadyForCone(){
        telemetry.addData("ARM", "Ready For Cone");
        armMotor.setTargetPosition(50);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.8);
    }
    public void SlowDown(){
        telemetry.addData("ARM", "Ready For Cone");
        armMotor.setTargetPosition(100);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.8);
    }

}

