package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoRightCloseFast extends CommandBase {

    private final TurretSubsystem turret;

    public TurretAutoRightCloseFast(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateAutoRightCloseFast();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtAutoRightCloseFast();
    }
}
