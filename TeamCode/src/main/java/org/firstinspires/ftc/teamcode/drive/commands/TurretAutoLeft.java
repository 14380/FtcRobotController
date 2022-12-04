package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoLeft extends CommandBase {

    private final TurretSubsystem turret;

    public TurretAutoLeft(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateAutoLeft();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtAutoLeft();
    }
}
