package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class RobotClawOpen extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private final SlideSubsystem slideSubsystem;
    private final ArmSubsystem armSubsystem;

    public RobotClawOpen(ClawSubsystem subsystem, ArmSubsystem arm, SlideSubsystem slide) {
        clawSubsystem = subsystem;
        armSubsystem =arm;
        slideSubsystem = slide;

        addRequirements(clawSubsystem, armSubsystem, slideSubsystem);
    }

    @Override
    public void initialize() {

        clawSubsystem.Open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
