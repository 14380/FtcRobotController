package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

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

    public RobotStateSubsytem(){

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
