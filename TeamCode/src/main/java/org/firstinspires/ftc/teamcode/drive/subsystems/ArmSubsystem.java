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
        leftServo.setPosition(0.76);
        rightServo.setPosition(0.76);
    }

    public void Mid(){
        leftServo.setPosition(0.58);
        rightServo.setPosition(0.58);

        telemetry.addData("ARM", "MID");
    }

    public boolean isAtTop(){
        return leftServo.getPosition() > 0.73 && rightServo.getPosition() > 0.73;
    }

    public boolean isAtMid()
    {
        // Return when the mid point is located

        return leftServo.getPosition() > 0.5 && rightServo.getPosition() > 0.5 && rightServo.getPosition() < 0.6 && leftServo.getPosition() < 0.6;
    }

    public boolean isAtCone(){
        // Return true when the arm is at the bottom.
        return leftServo.getPosition() < 0.3 && rightServo.getPosition() < 0.3;
    }

    public void JustIntaken(){
        telemetry.addData("ARM", "Ready For Intake");
        leftServo.setPosition(0.35);
        rightServo.setPosition(0.35);
    }

    public void ReadyForCone(){
        telemetry.addData("ARM", "Ready For Cone");
        leftServo.setPosition(0.27);
        rightServo.setPosition(0.27);
    }
}

