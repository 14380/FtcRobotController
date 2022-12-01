package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRight extends CommandBase {

    private final TurretSubsystem turret;

    public TurretRight(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateRight();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtRight();
    }
}