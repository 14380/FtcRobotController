package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmMoveHighCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmMoveHighCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.MoveTop();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtMoveTop();
    }
}
