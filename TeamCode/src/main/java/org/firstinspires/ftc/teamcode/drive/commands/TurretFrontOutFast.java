package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretFrontOutFast extends CommandBase {

    private final TurretSubsystem turret;

    public TurretFrontOutFast(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateToFrontFast();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtFrontFast();
    }
}
