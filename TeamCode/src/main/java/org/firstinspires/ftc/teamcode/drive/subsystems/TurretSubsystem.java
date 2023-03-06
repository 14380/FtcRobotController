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

        turretEnc.setPower(0.95);

    }

    public void RotateToRear(){

        telemetry.addData("ROTATE", "REAR");
       // turretMotor.setPosition(0.39);
        turretEnc.setTargetPosition(-2400);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.8);
    }

    public void RotateLeft(){

        telemetry.addData("ROTATE", "LEFT");

        turretEnc.setTargetPosition(1400);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.8);

    }

    public boolean IsAtAutoRight(){
        return true;//turretMotor.getPosition() > 0.25 && turretMotor.getPosition() < 0.4;
    }

    public void RotateAutoLeftClose(){

        telemetry.addData("ROTATE", "LEFT");

        turretEnc.setTargetPosition(1900);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(1);

    }
    public void RotateAutoRightClose(){

        turretEnc.setTargetPosition(-1900);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(1);
    }

    public void ManualLeft(double speed){
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(Math.abs(turretEnc.getCurrentPosition()) < 2700 && Math.abs(turretEnc.getCurrentPosition()) > 0){
            turretEnc.setPower(speed);

        }else{
            turretEnc.setPower(0);

        }
    }

    public void ManualRight(double speed){
        turretEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(Math.abs(turretEnc.getCurrentPosition()) < 2700 && Math.abs(turretEnc.getCurrentPosition()) > 0){
            turretEnc.setPower(-1 * speed);

        }else{
            turretEnc.setPower(0);

        }
    }

    public void RotateRight()
    {
        telemetry.addData("ROTATE", "RIGHT");


        turretEnc.setTargetPosition(-2300);

        turretEnc.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretEnc.setPower(0.8);

    }

    public boolean IsAtRear(){

        return Math.abs(turretEnc.getCurrentPosition()) > 2200 && Math.abs(turretEnc.getCurrentPosition()) < 2700;
    }

    public boolean IsAtRight(){

        return turretEnc.getCurrentPosition() < 0 && Math.abs(turretEnc.getCurrentPosition()) > 1200 && Math.abs(turretEnc.getCurrentPosition()) < 2400;
    }


    public boolean IsAtLeft(){

        return Math.abs(turretEnc.getCurrentPosition()) > 1200 && Math.abs(turretEnc.getCurrentPosition()) < 1500;
    }

    public boolean IsAtAutoLeftClose(){

        return Math.abs(turretEnc.getCurrentPosition()) > 1300;
    }

    public boolean IsAtAutoRightClose(){

        return Math.abs(turretEnc.getCurrentPosition()) > 1300;
    }

    public boolean IsAtFront(){

        return    (Math.abs(turretEnc.getCurrentPosition()) >= 0 && Math.abs(turretEnc.getCurrentPosition()) < 20);

    }


}
