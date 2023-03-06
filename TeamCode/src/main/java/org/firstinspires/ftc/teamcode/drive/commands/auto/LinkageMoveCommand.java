package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class LinkageMoveCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private final ArmSubsystem armSubsystem;
    private final SlideSubsystem slideSubsystem;
    private final RobotStateSubsytem stateSubsytem;
    private final double amountToMove;

    public LinkageMoveCommand(double moveAmount, ClawSubsystem subsystem, ArmSubsystem arm, SlideSubsystem slide, RobotStateSubsytem rState) {
        clawSubsystem = subsystem;
        armSubsystem = arm;
        slideSubsystem = slide;
        stateSubsytem = rState;
        amountToMove = moveAmount;

        addRequirements(clawSubsystem, armSubsystem, slideSubsystem, stateSubsytem);
    }

    @Override
    public void initialize() {
        stateSubsytem.setExtendoMode(RobotStateSubsytem.ExtenoState.OUT);
        clawSubsystem.LinkageMove(amountToMove);


    }

    @Override
    public boolean isFinished() {

        return true;
    }
}
