package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideUpTopCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;

    public SlideUpTopCommand(SlideSubsystem subsystem) {
        slideSubsystem = subsystem;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {

        slideSubsystem.SlideToTop();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideAtTop();
    }
}
