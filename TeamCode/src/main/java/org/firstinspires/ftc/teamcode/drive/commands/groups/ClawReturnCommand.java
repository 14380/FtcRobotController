package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.RobotSlideGraspCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class ClawReturnCommand extends SequentialCommandGroup {

    public ClawReturnCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            RobotStateSubsytem robotState)
    {


        addCommands(

                new ConditionalCommand(
                        new SequentialCommandGroup(
                            new RobotClawOpen(claw, arm, slide, robotState ),
                            new WaitCommand(200),
                            new ArmHelperInCommand(arm)
                        ),
                        new SequentialCommandGroup(

                                new RobotClawOpen(claw, arm, slide, robotState),
                                new WaitCommand(200),
                                new LinkageInCommand(claw, arm, slide, robotState),
                                new ArmHelperInCommand(arm)
                        ),
                        () -> {

                            return robotState.getExtendoState() == RobotStateSubsytem.ExtenoState.IN;
                        }
                ),


                new ConditionalCommand(
                        new SlideToConeCommand(slide, arm),
                        new InstantCommand(claw::Open),
                        () -> {

                            return slide.IsSlideAtGrasp();
                        }
                ));






        addRequirements( arm, slide, claw);
    }


}
