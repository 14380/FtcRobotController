package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class SlideToLinkageOutCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;
    private final ArmSubsystem armSubsystem;

    public SlideToLinkageOutCommand(SlideSubsystem subsystem, ArmSubsystem arm) {
        slideSubsystem = subsystem;
        armSubsystem = arm;
        addRequirements(slideSubsystem);
    }

    @Override
    public void initialize() {
       
        slideSubsystem.SlideToLinkageOut();
    }

    @Override
    public boolean isFinished() {
        return slideSubsystem.IsSlideToLinkageOut();
    }
}
