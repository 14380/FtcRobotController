package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class RobotClawClose extends CommandBase {

    private final ClawSubsystem clawSubsystem;
    private final ArmSubsystem armSubsystem;
    private final SlideSubsystem slideSubsystem;

    public RobotClawClose(ClawSubsystem subsystem, ArmSubsystem arm, SlideSubsystem slide) {
        clawSubsystem = subsystem;
        armSubsystem = arm;
        slideSubsystem = slide;
        addRequirements(clawSubsystem, armSubsystem, slideSubsystem);
    }

    @Override
    public void initialize() {

        clawSubsystem.Close();


    }

    @Override
    public boolean isFinished() {

        return clawSubsystem.IsClosed();
    }
}