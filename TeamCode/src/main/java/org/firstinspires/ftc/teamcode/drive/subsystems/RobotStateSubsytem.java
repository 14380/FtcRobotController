package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotStateSubsytem extends SubsystemBase {

    public enum ArmCollectionState{
        NORMAL,
        STACK
    }

    public enum SlideControlMode{
        AUTO,
        MANUAL
    }

    private ArmCollectionState armState = ArmCollectionState.NORMAL;
    private SlideControlMode slideState = SlideControlMode.AUTO;

    private RevBlinkinLedDriver blinkinLedDriver;

    public RobotStateSubsytem(HardwareMap hardwareMap){
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
    }

    public void setPattern(double elapsedTime){

        if(elapsedTime > 60){
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        else if(elapsedTime >= 40){
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        }
        else if(elapsedTime > 30){
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.HOT_PINK);
        }
        else if(elapsedTime > 5){
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        else{
            blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
        }
    }

    public void setArmState(ArmCollectionState state){
        this.armState = state;
    }

    public void setSlideState(SlideControlMode mode)
    {
        this.slideState = mode;
    }

    public ArmCollectionState getArmState(){
        return this.armState;
    }

    public SlideControlMode getSlideState(){
        return this.slideState;
    }

}
