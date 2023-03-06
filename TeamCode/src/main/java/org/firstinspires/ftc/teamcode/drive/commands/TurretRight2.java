package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRight2 extends CommandBase {

    private final TurretSubsystem turret;

    public TurretRight2(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

       // turret.RotateRight2();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
