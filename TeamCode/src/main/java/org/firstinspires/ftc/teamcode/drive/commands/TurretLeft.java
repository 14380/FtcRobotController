package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretLeft extends CommandBase {

    private final TurretSubsystem turret;

    public TurretLeft(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateLeft();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtLeft();
    }
}
