package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;


public class ArmMidCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final ClawSubsystem clawSubsystem;

    public ArmMidCommand(ArmSubsystem subsystem, ClawSubsystem cSubsystem) {
        armSubsystem = subsystem;
        clawSubsystem = cSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        clawSubsystem.PitchMid();
        armSubsystem.Mid();
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtMid();
    }
}
