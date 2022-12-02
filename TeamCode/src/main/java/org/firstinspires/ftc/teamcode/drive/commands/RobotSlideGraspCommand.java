package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


public class RobotSlideGraspCommand extends CommandBase {

    private final ArmSubsystem armSubsystem;
    private final SlideSubsystem slideSubsystem;
    private final ClawSubsystem clawSubsystem;
    private final RobotStateSubsytem rState;

    public RobotSlideGraspCommand(SlideSubsystem slide, ArmSubsystem subsystem, ClawSubsystem claw, RobotStateSubsytem robotState) {
        armSubsystem = subsystem;
        slideSubsystem = slide;
        clawSubsystem = claw;
        rState = robotState;

        addRequirements(armSubsystem, slideSubsystem, rState);
    }

    @Override
    public void initialize() {

        clawSubsystem.Close();
        if(slideSubsystem.IsSlideAtBottom()){
            if(rState.getArmState() == RobotStateSubsytem.ArmCollectionState.NORMAL) {
                slideSubsystem.SlideToGrasp();
            }else{
                slideSubsystem.SlideToMid1Stack();
            }
        }
    }

    @Override
    public boolean isFinished() {

        return slideSubsystem.IsAtGrasp();
    }
}
