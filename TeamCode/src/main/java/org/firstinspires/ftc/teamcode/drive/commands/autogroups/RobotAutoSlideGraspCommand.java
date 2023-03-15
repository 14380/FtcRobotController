package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class RobotAutoSlideGraspCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final SlideSubsystem slideSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final RobotStateSubsytem rState;

    public RobotAutoSlideGraspCommand(SlideSubsystem slide, ArmSubsystem subsystem, ClawSubsystem claw, RobotStateSubsytem robotState) {
        armSubsystem = subsystem;
        slideSubsystem = slide;
        clawSubsystem = claw;
        rState = robotState;

        addRequirements(armSubsystem, slideSubsystem, rState);
    }

    @Override
    public void initialize() {

        clawSubsystem.Close();


        slideSubsystem.SlideToGrasp();
        armSubsystem.Mid();


    }

    @Override
    public boolean isFinished() {

        return slideSubsystem.IsAtGrasp() && armSubsystem.isAtMid();
    }
}
