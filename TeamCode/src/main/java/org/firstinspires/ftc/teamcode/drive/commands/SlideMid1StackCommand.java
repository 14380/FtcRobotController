package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideMid1StackCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;

    public SlideMid1StackCommand(SlideSubsystem subsystem) {
        slideSubsystem = subsystem;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {

        slideSubsystem.SlideToMid1Stack();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideAtMidStack1();
    }
}
