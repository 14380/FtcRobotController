package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;


public class ArmHighAuto5Command extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final int armPosition;

    public ArmHighAuto5Command(int armPos, ArmSubsystem subsystem) {
        armSubsystem = subsystem;
        armPosition = armPos;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        armSubsystem.MidStack5(armPosition);

    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtMid5(armPosition);
    }
}
