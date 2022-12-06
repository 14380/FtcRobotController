package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;

public class AutoSlideModeCommand extends CommandBase {

    private final RobotStateSubsytem stateSubSystem;

    public AutoSlideModeCommand(RobotStateSubsytem subsystem) {
        stateSubSystem = subsystem;
        addRequirements(stateSubSystem);
    }

    @Override
    public void initialize() {

        stateSubSystem.setSlideState(RobotStateSubsytem.SlideControlMode.AUTO);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
