package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class RobotClawHighPitchCommand extends CommandBase {

    private final ClawSubsystem clawSubsystem;

    private final RobotStateSubsytem rState;

    public RobotClawHighPitchCommand(ClawSubsystem subsystem, RobotStateSubsytem robotState) {
        clawSubsystem = subsystem;

        rState = robotState;

        addRequirements(clawSubsystem, rState);
    }

    @Override
    public void initialize() {

        clawSubsystem.PitchMid();// .HighPitch();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
