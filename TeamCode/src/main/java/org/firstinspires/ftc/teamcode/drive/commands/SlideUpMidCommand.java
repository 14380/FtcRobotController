package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideUpMidCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;

    public SlideUpMidCommand(SlideSubsystem subsystem) {
        slideSubsystem = subsystem;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {

        slideSubsystem.SlideToMid();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideAtMid();
    }
}
