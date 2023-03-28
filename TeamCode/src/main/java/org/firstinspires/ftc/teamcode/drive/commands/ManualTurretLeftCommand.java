package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;


public class ManualTurretLeftCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;
    private final ArmSubsystem armSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final RobotStateSubsytem robotStateSys;

    public ManualTurretLeftCommand(SlideSubsystem subsystem, ArmSubsystem arm, TurretSubsystem turret, RobotStateSubsytem rState) {
        slideSubsystem = subsystem;
        armSubsystem = arm;
        robotStateSys = rState;
        turretSubsystem = turret;
        addRequirements(slideSubsystem, armSubsystem, turretSubsystem);
    }

    @Override
    public void execute() {
        robotStateSys.setTurretMode(RobotStateSubsytem.TurretControlMode.MANUAL);
        turretSubsystem.ManualLeft(0.4);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
