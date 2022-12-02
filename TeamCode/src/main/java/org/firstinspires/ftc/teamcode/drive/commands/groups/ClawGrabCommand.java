package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotSlideGraspCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRight;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class ClawGrabCommand extends SequentialCommandGroup {

    public ClawGrabCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            RobotStateSubsytem robotState)
    {


        addCommands(    new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(200),
                        new RobotSlideGraspCommand(slide, arm, claw, robotState)
        );



        addRequirements( arm, slide, claw);
    }


}
