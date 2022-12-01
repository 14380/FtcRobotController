package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;

public class DebugCommand extends CommandBase {

    private final VisionSubsystem vision;

    public DebugCommand(VisionSubsystem subsystem) {
        vision = subsystem;
        addRequirements(vision);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {

        vision.dump();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
