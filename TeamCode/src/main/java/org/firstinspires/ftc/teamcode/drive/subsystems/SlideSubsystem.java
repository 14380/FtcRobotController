package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SlideSubsystem extends SubsystemBase {


    private final DcMotorEx leftSlide;
    private final DcMotorEx rightSlide;
    private final Telemetry telemetry;

    public SlideSubsystem(HardwareMap map, Telemetry tele)
    {
        leftSlide = map.get(DcMotorEx.class, "leftSlide");
        rightSlide = map.get(DcMotorEx.class, "rightSlide");
        telemetry = tele;
    }

    public void SlideToTop(){
        telemetry.addData("SLIDE", "Top");
        leftSlide.setTargetPosition(2000);
        rightSlide.setTargetPosition(2000);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.9);
        rightSlide.setPower(0.9);
    }

    public void SlideToBottom(){
        telemetry.addData("SLIDE", "Bottom");

        leftSlide.setTargetPosition(0);
        rightSlide.setTargetPosition(0);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);
    }

    public void SlideToMid(){
        telemetry.addData("SLIDE", "MID");

        leftSlide.setTargetPosition(1600);
        rightSlide.setTargetPosition(1600);

        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setPower(0.8);
        rightSlide.setPower(0.8);

    }

    public boolean IsSlideAtTop(){
        //TODO: Is at the top
        return true;
    }

    public boolean IsSlideAtBottom(){
        //TODO: Is at the bottom
        return true;
    }

    public boolean IsSlideAtMid(){
        //TODO: Is at the mid
        return true;
    }

}
