package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideToMidAuto2Command extends CommandBase {

    private final SlideSubsystem slideSubsystem;

    public SlideToMidAuto2Command(SlideSubsystem subsystem) {
        slideSubsystem = subsystem;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {

        slideSubsystem.SlideToMidAuto2();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideAtMidAuto2();
    }
}
