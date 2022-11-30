package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideToConeCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;
    private final ArmSubsystem armSubsystem;

    public SlideToConeCommand(SlideSubsystem subsystem, ArmSubsystem arm) {
        slideSubsystem = subsystem;
        armSubsystem = arm;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {
       
        slideSubsystem.SlideToBottom();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideAtBottom();
    }
}
