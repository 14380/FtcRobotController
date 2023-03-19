package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInSmallCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHighPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHigherPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHomePitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class Stack5RightCloseClawGrabCommand extends SequentialCommandGroup {

    public Stack5RightCloseClawGrabCommand(
            int armPos,
            double linkagePos,
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            TurretSubsystem turret,
            RobotStateSubsytem robotState)
    {


        addCommands(

                        new RobotClawOpen(claw, arm,slide, robotState),
                       new ArmHighAuto5Command(armPos, arm),
                       new RobotAutoPitchCommand(0.6,claw,robotState),
                        new LinkageMoveCommand(linkagePos, claw, arm, slide, robotState),
                        new WaitCommand(550),
                        new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(250),
                        new RobotClawHomePitchCommand(claw,robotState),
                        new WaitCommand(200),
                        new LinkageInCommand(claw, arm, slide, robotState),
                        new WaitCommand(100),
                        new ArmHighAuto5Command(3450, arm),
                        new TurretLeftUpCloseFastAutoCommand(arm, slide, turret, claw, robotState),
                        new TurretLeftUpCloseAutoCommand(arm, slide, turret, claw, robotState),
                        new ArmHighAuto5Command(2600, arm),
                        new WaitCommand(300),
                        new LinkageMoveCommand(0.35, claw, arm, slide, robotState),
                        new ArmHelperOutCommand(arm),
                        //new RobotAutoPitchCommand(0.8,claw,robotState),
                        new RobotClawHighPitchCommand(claw, robotState),
                        new WaitCommand(550),
                        new RobotClawOpen(claw,arm,slide, robotState),
                        new LinkageInCommand(claw, arm, slide, robotState),
                        new WaitCommand(150),
                        new ArmHelperInCommand(arm),
                        new TurretRearDownAutoCommand(arm, slide, turret, claw, robotState),
                        new ArmHighAuto5Command(armPos, arm),
                        new WaitCommand(100)
        );



        addRequirements( arm, slide, claw);
    }


}
