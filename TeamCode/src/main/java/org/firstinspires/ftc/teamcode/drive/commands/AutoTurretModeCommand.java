package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;

public class AutoTurretModeCommand extends CommandBase {

    private final RobotStateSubsytem stateSubSystem;

    public AutoTurretModeCommand(RobotStateSubsytem subsystem) {
        stateSubSystem = subsystem;
        addRequirements(stateSubSystem);
    }

    @Override
    public void initialize() {

        stateSubSystem.setTurretMode(RobotStateSubsytem.TurretControlMode.AUTO);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
