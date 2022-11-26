package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;


public class ArmMidCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;

    public ArmMidCommand(ArmSubsystem subsystem) {
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
