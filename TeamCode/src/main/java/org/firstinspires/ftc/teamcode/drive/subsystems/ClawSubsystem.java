package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ClawSubsystem extends SubsystemBase {

    private final Servo clawServo;
    private final Telemetry telemetry;

    public ClawSubsystem(HardwareMap map, Telemetry tele)
    {
        clawServo = map.get(Servo.class, "clawServo");
        telemetry = tele;

    }

    public void Open(){
        telemetry.addData("CLAW", "OPEN");
        clawServo.setPosition(1);
    }

    public void Close(){
        telemetry.addData("CLAW", "CLOSE");

        clawServo.setPosition(0.39);
    }

    public boolean IsClosed(){
        return clawServo.getPosition() < 0.41;
    }

    public void Report(){
        telemetry.addData("CLAW", clawServo.getPosition());
    }

}
