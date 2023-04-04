package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretFrontOutAuto extends CommandBase {

    private final TurretSubsystem turret;

    public TurretFrontOutAuto(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateToFrontAuto();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtFront();
    }
}