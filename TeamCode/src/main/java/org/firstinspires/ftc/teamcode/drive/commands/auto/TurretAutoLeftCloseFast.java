package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoLeftCloseFast extends CommandBase {

    private final TurretSubsystem turret;

    public TurretAutoLeftCloseFast(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateAutoLeftCloseFast();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtAutoLeftCloseFast();
    }
}
