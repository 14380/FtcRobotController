package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;

public class RobotClawClose extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    public RobotClawClose(ClawSubsystem subsystem) {
        clawSubsystem = subsystem;
        addRequirements(clawSubsystem);
    }

    @Override
    public void initialize() {

        clawSubsystem.Close();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
