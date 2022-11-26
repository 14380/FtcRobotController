package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretFrontOut extends CommandBase {

    private final TurretSubsystem turret;

    public TurretFrontOut(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateToFront();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtFront();
    }
}
