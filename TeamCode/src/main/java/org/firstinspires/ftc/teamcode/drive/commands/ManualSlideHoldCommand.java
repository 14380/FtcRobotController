package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class ManualSlideHoldCommand extends CommandBase {

    private final SlideSubsystem slideSubsystem;
    private final ArmSubsystem armSubsystem;
    private final RobotStateSubsytem robotStateSys;

    public ManualSlideHoldCommand(SlideSubsystem subsystem, ArmSubsystem arm, RobotStateSubsytem rState) {
        slideSubsystem = subsystem;
        armSubsystem = arm;
        robotStateSys = rState;
        addRequirements(slideSubsystem);
    }

    @Override
    public void execute() {
       //only hold if in manual mode
        if(robotStateSys.getSlideState() == RobotStateSubsytem.SlideControlMode.MANUAL) {
            slideSubsystem.ManualSlideUp(0);
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
