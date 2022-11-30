package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class RobotSlideGraspCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final SlideSubsystem slideSubsystem;
    private final ClawSubsystem clawSubsystem;

    public RobotSlideGraspCommand(SlideSubsystem slide, ArmSubsystem subsystem, ClawSubsystem claw) {
        armSubsystem = subsystem;
        slideSubsystem = slide;
        clawSubsystem = claw;
        addRequirements(armSubsystem, slideSubsystem);
    }

    @Override
    public void initialize() {

        clawSubsystem.Close();
        if(slideSubsystem.IsSlideAtBottom()){
            slideSubsystem.SlideToGrasp();
        }
    }

    @Override
    public boolean isFinished() {

        return slideSubsystem.IsAtGrasp();
    }
}
