package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretFrontOutFastLeft extends CommandBase {

    private final TurretSubsystem turret;

    public TurretFrontOutFastLeft(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateToFrontFastLeft();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtFrontFast();
    }
}
