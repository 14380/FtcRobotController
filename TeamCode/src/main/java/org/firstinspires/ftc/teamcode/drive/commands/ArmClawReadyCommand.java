package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;


public class ArmClawReadyCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final TurretSubsystem turretSubsystem;

    public ArmClawReadyCommand(ArmSubsystem subsystem, TurretSubsystem turret) {
        armSubsystem = subsystem;
        turretSubsystem = turret;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if(turretSubsystem.IsAtFront()) {
            armSubsystem.ReadyForCone();
        }
    }

    @Override
    public boolean isFinished() {
       // return true;

        return armSubsystem.isAtCone();
    }
}
