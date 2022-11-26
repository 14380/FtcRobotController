package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;

public class RobotClawOpen extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    public RobotClawOpen(ClawSubsystem subsystem) {
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.Open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
