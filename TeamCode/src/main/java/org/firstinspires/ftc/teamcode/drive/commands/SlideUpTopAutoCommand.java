package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideUpTopAutoCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;

    public SlideUpTopAutoCommand(SlideSubsystem subsystem) {
        slideSubsystem = subsystem;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {

        slideSubsystem.SlideToTop2();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideAtTop2();
    }
}
