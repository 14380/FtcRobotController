package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoFMLeft extends CommandBase {

    private final TurretSubsystem turret;

    public TurretAutoFMLeft(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateAutoLeftFM();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtAutoLeft();
    }
}
