package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmHelperInCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmHelperInCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.HelperIn();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
