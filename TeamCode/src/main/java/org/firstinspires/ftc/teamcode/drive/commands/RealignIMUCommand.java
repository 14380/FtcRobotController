package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;

public class RealignIMUCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    public RealignIMUCommand(DriveSubsystem subsystem) {
        driveSubsystem = subsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        driveSubsystem.Realign();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
