package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

public class RobotStateSubsytem extends SubsystemBase {

    public enum ArmCollectionState{
        NORMAL,
        STACK
    }

    private ArmCollectionState armState = ArmCollectionState.NORMAL;

    public RobotStateSubsytem(){

    }

    public void setArmState(ArmCollectionState state){
        this.armState = state;
    }

    public ArmCollectionState getArmState(){
        return this.armState;
    }

}
