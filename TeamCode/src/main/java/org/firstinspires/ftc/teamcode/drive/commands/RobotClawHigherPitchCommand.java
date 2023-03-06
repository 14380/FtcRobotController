package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;

public class RobotClawHigherPitchCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    private final RobotStateSubsytem rState;

    public RobotClawHigherPitchCommand(ClawSubsystem subsystem, RobotStateSubsytem robotState) {
        clawSubsystem = subsystem;

        rState = robotState;

        addRequirements(clawSubsystem, rState);
    }

    @Override
    public void initialize() {

        clawSubsystem.HighPitch();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
