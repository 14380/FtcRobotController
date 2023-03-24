package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmHelperTeleOpOutCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmHelperTeleOpOutCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.HelperOutTeleOp();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
