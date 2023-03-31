package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretFrontOutAutoLeft extends CommandBase {

    private final TurretSubsystem turret;

    public TurretFrontOutAutoLeft(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateToFrontAutoLeft();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtFrontAutoLeft();
    }
}
