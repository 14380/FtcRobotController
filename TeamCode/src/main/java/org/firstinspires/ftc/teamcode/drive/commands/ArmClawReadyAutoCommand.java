package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;


public class ArmClawReadyAutoCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final RobotStateSubsytem rState;

    public ArmClawReadyAutoCommand(ArmSubsystem subsystem, RobotStateSubsytem robotState) {
        armSubsystem = subsystem;

        rState = robotState;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

            armSubsystem.ReadyForCone();

    }

    @Override
    public boolean isFinished() {
       // return true;

        return armSubsystem.isAtConeAutoCheck();
    }
}
