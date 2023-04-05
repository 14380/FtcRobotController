package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class AutoClawGrabStartRightMedCommand extends SequentialCommandGroup {

    public AutoClawGrabStartRightMedCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            TurretSubsystem turret,
            RobotStateSubsytem robotState)
    {


        addCommands(
                        new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(150),
                        new RobotAutoSlideGraspCommand(slide, arm, claw, robotState),


                        new TurretLeftUpFirstCloseAutoCommand(arm, slide, turret, claw, robotState),

                        new SlideToConeCommand(slide, arm),

                        new RobotAutoPitchCommand(0.80,claw,robotState),

                        new ArmHighAuto5Command(2650, arm)







        );



        addRequirements( arm, slide, claw);
    }


}
