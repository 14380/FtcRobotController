package org.firstinspires.ftc.teamcode.drive.commands.auto;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretAutoRightClose extends CommandBase {

    private final TurretSubsystem turret;

    public TurretAutoRightClose(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateAutoRightClose();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtAutoRightClose();
    }
}
