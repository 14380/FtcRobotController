package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TurretSubsystem extends SubsystemBase {

    private final Servo turretMotor;
    private final Telemetry telemetry;
    private final DcMotorEx turretEnc;

    public TurretSubsystem(HardwareMap map, Telemetry tele)
    {
        turretMotor = map.get(Servo.class, "turretServo");
        turretEnc = map.get(DcMotorEx.class, "turretEnc");
        telemetry = tele;
    }

    public void RotateToFront()
    {
        //need to make sure we rotate the correct way
        telemetry.addData("ROTATE", "FRONT");


        turretMotor.setPosition(0.51);


    }

    public void RotateToRear(){

        telemetry.addData("ROTATE", "REAR");
        turretMotor.setPosition(0.38);
    }

    public void RotateLeft(){

        telemetry.addData("ROTATE", "LEFT");

        turretMotor.setPosition(0.6);
    }

    public void RotateRight()
    {
        telemetry.addData("ROTATE", "RIGHT");


        turretMotor.setPosition(0.43);

    }

    public boolean IsAtRear(){

        return true;
    }

    public boolean IsAtRight(){

        return turretMotor.getPosition() > 0.3 && turretMotor.getPosition() < 0.5;
    }

    public boolean IsAtLeft(){

        return turretMotor.getPosition() > 0.55 && turretMotor.getPosition() < 0.8;
    }

    public boolean IsAtFront(){

        return
                (Math.abs(turretEnc.getCurrentPosition()) >= 0 && Math.abs(turretEnc.getCurrentPosition()) < 300);

    }


}