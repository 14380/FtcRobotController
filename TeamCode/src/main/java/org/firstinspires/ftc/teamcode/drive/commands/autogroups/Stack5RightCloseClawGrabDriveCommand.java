package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class Stack5RightCloseClawGrabDriveCommand extends SequentialCommandGroup {

    public Stack5RightCloseClawGrabDriveCommand(
            int armPos,
            double linkagePos,
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            TurretSubsystem turret,
            TrajectorySequenceFollowerCommand movetoJunc,
            TrajectorySequenceFollowerCommand movetoCone,
            RobotStateSubsytem robotState, boolean isFinal)
    {


        addCommands(
                        new ParallelCommandGroup(
                                movetoCone,
                                new SequentialCommandGroup(
                                        new RobotClawOpen(claw, arm,slide, robotState),
                                        new RobotAutoPitchCommand(0.1,claw,robotState),
                                        new WaitCommand(200),
                                        new ArmClawReadyAutoCommand(arm, robotState)
                                )
                        ),


                        new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(240),

                        new RobotAutoPitchCommand(0.45,claw,robotState), //0.3
                        new SlideToMidCommand(slide),
                        new ParallelCommandGroup(

                                        movetoJunc,

                                new SequentialCommandGroup(

                                        new LinkageInCommand(claw, arm, slide, robotState),
                                        new WaitCommand(100),
                                        new SlideToConeCommand(slide, arm),
                                        new ArmHighAuto5Command(2550, arm),
                                        new TurretLeftUpCloseFastAutoCommand(arm, slide, turret, claw, robotState),
                                        new TurretLeftUpCloseAutoCommand(arm, slide, turret, claw, robotState),
                                        new ArmHighAuto5Command(2650, arm)

                                )
                        ),
                        new ArmHelperOutCommand(arm),
                        new WaitCommand(300),
                        new LinkageMoveCommand(0.31, claw, arm, slide, robotState),
                        new WaitCommand(100),
                        new RobotAutoPitchCommand(0.75, claw, robotState), //0.7

                        new RobotClawOpen(claw,arm,slide, robotState),

                        new WaitCommand(100),
                        new ArmHelperInCommand(arm),
                        new WaitCommand(200),
                        new LinkageInCommand(claw, arm, slide, robotState),
                        new RobotAutoPitchCommand(0.1,claw,robotState),

                        new ConditionalCommand(

                                new WaitCommand(1),// ArmClawReadyCommand(arm,turret, robotState),
                                new SequentialCommandGroup(
                                    new TurretRearDownAutoFastCommand(arm, slide, turret, claw, robotState),
                                    new TurretRearDownAutoCommand(arm, slide, turret, claw, robotState),
                                    new ArmClawReadyAutoCommand(arm, robotState)
                                ),
                                () -> {return isFinal;}
                        )

        );




        addRequirements( arm, slide, claw);
    }


}
