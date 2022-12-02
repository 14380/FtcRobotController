package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;


public class ArmStackMid1Command extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final RobotStateSubsytem rState;

    public ArmStackMid1Command(ArmSubsystem subsystem, RobotStateSubsytem robotState) {
        armSubsystem = subsystem;
        this.rState = robotState;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        rState.setArmState(RobotStateSubsytem.ArmCollectionState.STACK);
        armSubsystem.MidStack1();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtMidStack1();
    }
}
