package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
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


        addCommands(    //new ArmClawReadyCommand(arm, turret,robotState),
                        new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(250),
                        new RobotAutoSlideGraspCommand(slide, arm, claw, robotState),
                        new TurretLeftUpCloseAutoCommand(arm, slide, turret, claw, robotState),
                        new ArmHighAuto5Command(2650, arm),
                        new SlideToConeCommand(slide, arm),

                        new LinkageMoveCommand(0.35, claw, arm, slide, robotState),
                        new ArmHelperOutCommand(arm),
                        new RobotAutoPitchCommand(0.8,claw,robotState),
                        new WaitCommand(850),
                        new RobotClawOpen(claw,arm,slide, robotState),
                        new WaitCommand(250),
                        new ArmHelperInCommand(arm),
                        new LinkageInCommand(claw, arm, slide, robotState),
                        new TurretRearDownAutoCommand(arm, slide, turret, claw, robotState),
                        new ArmHighAuto5Command(1250, arm),
                        new WaitCommand(100)


        );



        addRequirements( arm, slide, claw);
    }


}
