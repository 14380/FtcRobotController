package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class LinkageOutCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private final ArmSubsystem armSubsystem;
    private final SlideSubsystem slideSubsystem;
    private final RobotStateSubsytem stateSubsytem;

    public LinkageOutCommand(ClawSubsystem subsystem, ArmSubsystem arm, SlideSubsystem slide, RobotStateSubsytem rState) {
        clawSubsystem = subsystem;
        armSubsystem = arm;
        slideSubsystem = slide;
        stateSubsytem = rState;

        addRequirements(clawSubsystem, armSubsystem, slideSubsystem, stateSubsytem);
    }

    @Override
    public void initialize() {
        stateSubsytem.setExtendoMode(RobotStateSubsytem.ExtenoState.OUT);
        clawSubsystem.LinkageOut();


    }

    @Override
    public boolean isFinished() {

        return true;
    }
}
