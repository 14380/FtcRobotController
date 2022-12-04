package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideToMidAutoCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;

    public SlideToMidAutoCommand(SlideSubsystem subsystem) {
        slideSubsystem = subsystem;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {

        slideSubsystem.SlideToMidAuto();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideAtMidAuto();
    }
}
