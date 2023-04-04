package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperTeleOpOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidAuto2Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class Stack5RightContestedAutoCommand extends SequentialCommandGroup {

    public Stack5RightContestedAutoCommand(
            int armPos,
            double linkagePos,
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            TurretSubsystem turret,
            RobotStateSubsytem robotState, boolean isFinal)
    {


        addCommands(
                        new WaitCommand(50),
                        new RobotClawOpen(claw, arm,slide, robotState),
                        new ArmHighAuto5Command(armPos, arm),
                        new WaitCommand(100),
                        new RobotAutoPitchCommand(0.6,claw,robotState),
                        new LinkageMoveCommand(linkagePos, claw, arm, slide, robotState),
                        new WaitCommand(320),
                        new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(350),
                        //new RobotClawHomePitchCommand(claw,robotState),
                        new RobotAutoPitchCommand(0.3,claw,robotState),
                        new WaitCommand(100),
                        new LinkageInCommand(claw, arm, slide, robotState),
                        new ParallelCommandGroup(
                                new SlideToMidAuto2Command(slide),
                                new ArmHighAuto5Command(2500, arm)
                        ),

                        new TurretRightUpContestedFastAutoCommand(arm, slide, turret, claw, robotState),
                        //new TurretRightUpContestedAutoCommand(arm, slide, turret, claw, robotState),

                        new ArmHelperTeleOpOutCommand(arm),
                        new WaitCommand(330),
                        new LinkageMoveCommand(0.22, claw, arm, slide, robotState),

                        new RobotAutoPitchCommand(0.7, claw, robotState), //0.7

                        new WaitCommand(400),

                        new RobotClawOpen(claw,arm,slide, robotState),

                        new WaitCommand(100),
                        new ArmHelperInCommand(arm),
                        //new WaitCommand(200),
                        new LinkageInCommand(claw, arm, slide, robotState),
                       // new ArmHighAuto5Command(2800, arm),
                        //new TurretRearDownAutoFastCommand(arm, slide, turret, claw, robotState),

                        new TurretRearDownAutoHighCommand(arm, slide, turret, claw, robotState),
                        new ArmHighAuto5Command(armPos, arm),

                        new ConditionalCommand(

                                new WaitCommand(1),// ArmClawReadyCommand(arm,turret, robotState),
                                new ArmHighAuto5Command(armPos, arm),
                                () -> {return isFinal;}
                        )

        );




        addRequirements( arm, slide, claw);
    }


}