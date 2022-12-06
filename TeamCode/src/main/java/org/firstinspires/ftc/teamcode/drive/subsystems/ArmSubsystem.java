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
        leftServo.setPosition(0.71);
        rightServo.setPosition(0.71);
    }

    public void TopAuto(){
        telemetry.addData("ARM", "TOP AUTO");
        leftServo.setPosition(0.65);
        rightServo.setPosition(0.65);
    }


    public void TopAuto2(){
        telemetry.addData("ARM", "TOP AUTO 2");
        leftServo.setPosition(0.67);
        rightServo.setPosition(0.67);
    }

    public void Mid(){
        leftServo.setPosition(0.54);
        rightServo.setPosition(0.54);

        telemetry.addData("ARM", "MID");
    }
    //the stack
    public void MidStack1(){
        leftServo.setPosition(0.43);
        rightServo.setPosition(0.43);

        telemetry.addData("ARM", "MID Stack 1");
    }

    public void MidStack2(){
        leftServo.setPosition(0.38);
        rightServo.setPosition(0.38);

        telemetry.addData("ARM", "MID Stack 2");
    }

    public boolean isAtTop(){
        return leftServo.getPosition() > 0.68 && rightServo.getPosition() > 0.68;
    }

    public boolean isAtTopAuto(){
        return leftServo.getPosition() > 0.5 && rightServo.getPosition() > 0.5;
    }

    public boolean isAtMid()
    {
        // Return when the mid point is located

        return leftServo.getPosition() > 0.45 && rightServo.getPosition() > 0.45 && rightServo.getPosition() < 0.6 && leftServo.getPosition() < 0.6;
    }

    public boolean isAtMidStack1()
    {
        // Return when the mid point is located

        return leftServo.getPosition() > 0.3 && rightServo.getPosition() > 0.3 && rightServo.getPosition() < 0.45 && leftServo.getPosition() < 0.45;
    }

    public boolean isAtMidStack2()
    {
        // Return when the mid point is located

        return leftServo.getPosition() > 0.3 && rightServo.getPosition() > 0.3 && rightServo.getPosition() < 0.5 && leftServo.getPosition() < 0.5;
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
        leftServo.setPosition(0.20);
        rightServo.setPosition(0.20);
    }
}

