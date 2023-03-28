package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;

public class AutoArmHeightModeCommand extends CommandBase {

    private final RobotStateSubsytem stateSubSystem;
    private final RobotStateSubsytem.ArmHeightPosition position;

    public AutoArmHeightModeCommand(RobotStateSubsytem.ArmHeightPosition position, RobotStateSubsytem subsystem) {
        stateSubSystem = subsystem;
        this.position = position;
        addRequirements(stateSubSystem);
    }

    @Override
    public void initialize() {

        stateSubSystem.setArmHightPosition(this.position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
