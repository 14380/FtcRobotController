package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class RobotClawOpen extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private final SlideSubsystem slideSubsystem;
    private final ArmSubsystem armSubsystem;
    private final RobotStateSubsytem rState;

    public RobotClawOpen(ClawSubsystem subsystem, ArmSubsystem arm, SlideSubsystem slide, RobotStateSubsytem robotState) {
        clawSubsystem = subsystem;
        armSubsystem =arm;
        slideSubsystem = slide;
        rState = robotState;

        addRequirements(clawSubsystem, armSubsystem, slideSubsystem, rState);
    }

    @Override
    public void initialize() {
       // rState.setArmState(RobotStateSubsytem.ArmCollectionState.NORMAL);
        clawSubsystem.Open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
