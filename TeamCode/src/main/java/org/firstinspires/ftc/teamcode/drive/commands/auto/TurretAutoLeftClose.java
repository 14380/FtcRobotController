package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoLeftClose extends CommandBase {

    private final TurretSubsystem turret;

    public TurretAutoLeftClose(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateAutoLeftClose();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtAutoLeftClose();
    }
}
