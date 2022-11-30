package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmStateCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmStateCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.Mid();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtMid();
    }
}
