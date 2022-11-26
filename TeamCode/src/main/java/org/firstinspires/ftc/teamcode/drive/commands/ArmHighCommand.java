package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmHighCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmHighCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.Top();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtTop();
    }
}
