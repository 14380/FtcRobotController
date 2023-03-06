package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;


public class ArmClawSlowCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final RobotStateSubsytem rState;

    public ArmClawSlowCommand(ArmSubsystem subsystem, ClawSubsystem claw, TurretSubsystem turret, RobotStateSubsytem robotState) {
        armSubsystem = subsystem;
        turretSubsystem = turret;
        clawSubsystem = claw;
        rState = robotState;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        if(turretSubsystem.IsAtFront()) {
            rState.setArmState(RobotStateSubsytem.ArmCollectionState.NORMAL);
            clawSubsystem.PitchUp();
            armSubsystem.SlowDown();
        }
    }

    @Override
    public boolean isFinished() {
       // return true;

        return true;
    }
}
