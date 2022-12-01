package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {

    private final VisionSubsystem vision;

    public VisionCommand(VisionSubsystem subsystem) {
        vision = subsystem;
        addRequirements(vision);
    }

    @Override
    public void initialize() {

        vision.periodic();
    }

    @Override
    public void execute() {

        vision.periodic();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
