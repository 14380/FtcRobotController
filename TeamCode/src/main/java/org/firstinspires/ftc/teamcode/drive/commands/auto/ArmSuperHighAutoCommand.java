package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmSuperHighAutoCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmSuperHighAutoCommand(ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.SuperTopAuto();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtTopAuto();
    }
}
