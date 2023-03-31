package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoPosition extends CommandBase {

    private final TurretSubsystem turret;
    private final int turretPosition;

    public TurretAutoPosition(int position,TurretSubsystem subsystem) {
        turret = subsystem;
        turretPosition = position;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotatePosition(turretPosition);
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtPosition(turretPosition - 20, turretPosition + 20);
    }
}
