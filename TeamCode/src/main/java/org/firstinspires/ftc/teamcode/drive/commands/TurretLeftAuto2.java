package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretLeftAuto2 extends CommandBase {

    private final TurretSubsystem turret;

    public TurretLeftAuto2(TurretSubsystem subsystem) {
        turret = subsystem;
        addRequirements(turret);
    }

    @Override
    public void initialize() {

        turret.RotateLeftAuto2();
    }

    @Override
    public boolean isFinished() {
        return turret.IsAtLeft();
    }
}
