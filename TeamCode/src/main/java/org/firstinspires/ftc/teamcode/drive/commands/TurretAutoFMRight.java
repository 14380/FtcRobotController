package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoFMRight extends CommandBase {

    private final TurretSubsystem turret;

    public TurretAutoFMRight(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

       // turret.RotateAutoRightFM();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtAutoRight();
    }
}
