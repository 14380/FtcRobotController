package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;


public class ArmClawReadyCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final RobotStateSubsytem rState;

    public ArmClawReadyCommand(ArmSubsystem subsystem, TurretSubsystem turret, RobotStateSubsytem robotState) {
        armSubsystem = subsystem;
        turretSubsystem = turret;
        rState = robotState;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if(turretSubsystem.IsAtFront()) {
            rState.setArmState(RobotStateSubsytem.ArmCollectionState.NORMAL);
            armSubsystem.ReadyForCone();
        }
    }

    @Override
    public boolean isFinished() {
       // return true;

        return armSubsystem.isAtCone();
    }
}
