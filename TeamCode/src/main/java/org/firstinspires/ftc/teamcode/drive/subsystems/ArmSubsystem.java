package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubsystem extends SubsystemBase {

    private final Servo leftServo;
    private final Servo rightServo;
    private final Telemetry telemetry;

    public ArmSubsystem(HardwareMap map, Telemetry tele)
    {
        leftServo = map.get(Servo.class, "leftServo");
        rightServo = map.get(Servo.class, "rightServo");
        telemetry = tele;
    }

    public void Top(){
        telemetry.addData("ARM", "TOP");
        leftServo.setPosition(0.4);
        rightServo.setPosition(0.4);
    }

    public void Mid(){
        leftServo.setPosition(0.27);
        rightServo.setPosition(0.27);

        telemetry.addData("ARM", "MID");
    }

    public boolean isAtTop(){
        return leftServo.getPosition() > 0.35 && rightServo.getPosition() > 0.35;
    }

    public boolean isAtMid()
    {
        //TODO: Return when the mid point is located
        //return true;
        return leftServo.getPosition() > 0.2 && rightServo.getPosition() > 0.2 && rightServo.getPosition() < 0.3 && leftServo.getPosition() < 0.3;
    }

    public boolean isAtCone(){
        //TODO: Return true when the arm is at the bottom.
        return leftServo.getPosition() < 0.1 && rightServo.getPosition() < 0.1;
    }
    public void ReadyForCone(){
        telemetry.addData("ARM", "Ready For Cone");
        leftServo.setPosition(0.02);
        rightServo.setPosition(0.02);
    }
}

