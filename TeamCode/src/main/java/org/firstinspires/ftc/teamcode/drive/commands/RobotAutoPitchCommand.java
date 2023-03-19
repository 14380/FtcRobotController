package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;

public class RobotAutoPitchCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    private final RobotStateSubsytem rState;

    private double pitch;

    public RobotAutoPitchCommand(double pos, ClawSubsystem subsystem, RobotStateSubsytem robotState) {
        clawSubsystem = subsystem;

        rState = robotState;

        pitch = pos;

        addRequirements(clawSubsystem, rState);
    }

    @Override
    public void initialize() {

        clawSubsystem.PitchAuto(pitch);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
