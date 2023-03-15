package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;


public class ArmClawHitCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final RobotStateSubsytem rState;

    public ArmClawHitCommand(ArmSubsystem subsystem, TurretSubsystem turret, RobotStateSubsytem robotState) {
        armSubsystem = subsystem;
        turretSubsystem = turret;
        rState = robotState;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if(turretSubsystem.IsAtFront()) {
            rState.setArmState(RobotStateSubsytem.ArmCollectionState.NORMAL);
            
            armSubsystem.RunToHitTurret();
        }
    }

    @Override
    public boolean isFinished() {
       // return true;

        return armSubsystem.isAtCone();
    }
}
