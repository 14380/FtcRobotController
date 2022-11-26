package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {

    private final DcMotorEx turretMotor;
    private final Telemetry telemetry;

    public TurretSubsystem(HardwareMap map, Telemetry tele)
    {
        turretMotor = map.get(DcMotorEx.class, "turretMotor");
        telemetry = tele;
    }

    public void RotateToFront()
    {
        //need to make sure we rotate the correct way
        telemetry.addData("ROTATE", "FRONT");
        turretMotor.setTargetPosition(0);

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor.setPower(0.5);


    }

    public void RotateToRear(){

        telemetry.addData("ROTATE", "REAR");
    }

    public void RotateLeft(){
        //TODO: Lets get the other stuff working first.
        telemetry.addData("ROTATE", "LEFT");
        turretMotor.setTargetPosition(-320);

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor.setPower(0.5);
    }

    public void RotateRight()
    {
        telemetry.addData("ROTATE", "RIGHT");

        turretMotor.setTargetPosition(320);

        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turretMotor.setPower(0.5);

    }

    public boolean IsAtRear(){
        //TODO: Check encoders
        return true;
    }

    public boolean IsAtRight(){
        //TODO: Check encoders.
        return turretMotor.getCurrentPosition() > 320;
    }

    public boolean IsAtLeft(){
        //TODO: Check encoders.
        return turretMotor.getCurrentPosition() < -320;
    }

    public boolean IsAtFront(){
        //TODO: Check encoders.
        return turretMotor.getCurrentPosition() < 20 && turretMotor.getCurrentPosition() > -20;
    }


}
