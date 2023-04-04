package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidAuto2Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class Stack5RightCloseClawGrabSlides extends SequentialCommandGroup {

    public Stack5RightCloseClawGrabSlides(
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
                        new SlideToMidAuto2Command(slide),
                        new RobotClawOpen(claw, arm,slide, robotState),
                        new ArmHighAuto5Command(armPos, arm),
                        new WaitCommand(100),
                        new RobotAutoPitchCommand(0.65,claw,robotState), //0.6
                        new LinkageMoveCommand(linkagePos, claw, arm, slide, robotState),
                        new WaitCommand(320),
                        new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(340),
                        new RobotAutoPitchCommand(0.45,claw,robotState), //0.3
                        new LinkageInCommand(claw, arm, slide, robotState),
                        new WaitCommand(100),
                        new ArmHighAuto5Command(1700, arm),
                        new TurretLeftUpCloseFastAutoCommand(arm, slide, turret, claw, robotState),
                        new TurretLeftUpCloseAutoCommand(arm, slide, turret, claw, robotState),
                        new ArmHighAuto5Command(1800, arm),
                        new ArmHelperOutCommand(arm),
                        new WaitCommand(300),
                        new LinkageMoveCommand(0.31, claw, arm, slide, robotState),
                        new WaitCommand(100),
                        new RobotAutoPitchCommand(0.75, claw, robotState), //0.7

                        new WaitCommand(500),

                        new RobotClawOpen(claw,arm,slide, robotState),

                        new WaitCommand(100),
                        new ArmHelperInCommand(arm),
                        new WaitCommand(200),
                        new LinkageInCommand(claw, arm, slide, robotState),


                        new ConditionalCommand(

                                new WaitCommand(1),// ArmClawReadyCommand(arm,turret, robotState),
                                new SequentialCommandGroup(
                                    new TurretRearDownAutoFastCommand(arm, slide, turret, claw, robotState),
                                    new TurretRearDownAutoCommand(arm, slide, turret, claw, robotState),
                                    new ArmHighAuto5Command(armPos, arm)
                                ),
                                () -> {return isFinal;}
                        )

        );




        addRequirements( arm, slide, claw);
    }


}
