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
    private ServoImplEx linkageServo2;
    private final Telemetry telemetry;

    public ClawSubsystem(HardwareMap map, Telemetry tele)
    {
        clawServo = map.get(Servo.class, "clawServo");
        pitchClaw = map.get(Servo.class, "clawVertServo");
        linkageServo = map.get(ServoImplEx.class, "linkageServo");
        linkageServo2 = map.get(ServoImplEx.class, "linkage2Servo");
        telemetry = tele;
        linkageServo.setPwmRange(new PwmControl.PwmRange(505, 2495));
        linkageServo2.setPwmRange(new PwmControl.PwmRange(505, 2495));
        //linkageServo2.setDirection(Servo.Direction.REVERSE);
        //linkageServo.setDirection(Servo.Direction.REVERSE);
    }

    public void Open(){
        telemetry.addData("CLAW", "OPEN");
        //pitchClaw.setPosition(0);
        clawServo.setPosition(0.9);
    }

    public void Close(){
        telemetry.addData("CLAW", "CLOSE");
       // pitchClaw.setPosition(0);
        clawServo.setPosition(0.6); //0.39);
    }
    public void PitchUp(){
        pitchClaw.setPosition(0.43); //0.53 //home pitch
    }

    public void PitchMid(){
        pitchClaw.setPosition(0.7); //up pos pitch
    }

    public void PitchAuto(double position){
        pitchClaw.setPosition(position);
    }
    public void LinkageIn(){

        linkageServo.setPosition(1);
        linkageServo2.setPosition(1);

    }

    public void LinkageOut(){
        linkageServo.setPosition(0.4);
        linkageServo2.setPosition(0.4);
    }

    public void LinkageInSmall(){

        linkageServo.setPosition(0.3);
        linkageServo2.setPosition(0.3);
    }

    public void LinkageMove(double amount){

        linkageServo.setPosition(amount);
        linkageServo2.setPosition(amount);
    }

    public void HighPitch(){
        pitchClaw.setPosition(0.2);
    }

    public void TopConePitch(){
        pitchClaw.setPosition(0.7);
    }

    public boolean IsClosed(){
        return true;
    }

    public void Report(){
        telemetry.addData("CLAW", clawServo.getPosition());
    }

}
