package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmMidAutoCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmMidAutoCommand(ArmSubsystem subsystem) {
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
