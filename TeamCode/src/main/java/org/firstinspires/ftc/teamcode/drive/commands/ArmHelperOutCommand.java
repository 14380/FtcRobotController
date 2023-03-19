package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmHelperOutCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmHelperOutCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.HelperOut();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
