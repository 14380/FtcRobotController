package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase {

    private final Servo clawServo;
    private Servo pitchClaw;
    private ServoImplEx linkageServo;
    private final Telemetry telemetry;

    public ClawSubsystem(HardwareMap map, Telemetry tele)
    {
        clawServo = map.get(Servo.class, "clawServo");
        pitchClaw = map.get(Servo.class, "clawVertServo");
        linkageServo = map.get(ServoImplEx.class, "linkageServo");
        telemetry = tele;
        linkageServo.setPwmRange(new PwmControl.PwmRange(505, 2495));

    }

    public void Open(){
        telemetry.addData("CLAW", "OPEN");
        //pitchClaw.setPosition(0);
        clawServo.setPosition(1);
    }

    public void Close(){
        telemetry.addData("CLAW", "CLOSE");
       // pitchClaw.setPosition(0);
        clawServo.setPosition(0.39);
    }
    public void PitchUp(){
        pitchClaw.setPosition(0);
    }

    public void PitchMid(){
        pitchClaw.setPosition(0.1);
    }
    public void LinkageOut(){

        linkageServo.setPosition(0.8);

    }

    public void LinkageIn(){
        linkageServo.setPosition(0);
    }

    public void LinkageInSmall(){
        linkageServo.setPosition(0.7);
    }

    public void LinkageMove(double amount){
        linkageServo.setPosition(amount);
    }

    public void HighPitch(){
        pitchClaw.setPosition(0.2);
    }

    public boolean IsClosed(){
        return clawServo.getPosition() < 0.41;
    }

    public void Report(){
        telemetry.addData("CLAW", clawServo.getPosition());
    }

}
