package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.RobotSlideGraspCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

public class ClawReturnCommand extends SequentialCommandGroup {

    public ClawReturnCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw)
    {


        addCommands(

                new RobotClawOpen(claw, arm, slide ),

                new ConditionalCommand(
                        new SlideToConeCommand(slide, arm),
                        new InstantCommand(claw::Open),
                        () -> {

                            return !slide.IsSlideAtTop() && !slide.IsSlideAtMid();
                        }
                ));





        addRequirements( arm, slide, claw);
    }


}
