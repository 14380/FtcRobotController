package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmClawReadyCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmClawReadyCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.ReadyForCone();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtCone();
    }
}
