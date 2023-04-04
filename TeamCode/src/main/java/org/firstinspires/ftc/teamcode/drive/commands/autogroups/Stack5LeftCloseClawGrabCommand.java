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
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class Stack5LeftCloseClawGrabCommand extends SequentialCommandGroup {

    public Stack5LeftCloseClawGrabCommand(
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
                        new ConditionalCommand(
                                new WaitCommand(350),
                                new WaitCommand(690),
                                ()->{
                                    return isFinal;
                                }
                        ),
                        new RobotAutoPitchCommand(0.70,claw,robotState), //higher value, more parrell ot ground .this angle is the angle the claw comes into the cone

                new LinkageMoveCommand(linkagePos, claw, arm, slide, robotState), //linkage out, into the cone

                //picking up off the stack
                new WaitCommand(290),
                new RobotClawClose(claw, arm, slide ),
                new ConditionalCommand(
                        new WaitCommand(200),
                        new WaitCommand(250),
                        ()->{
                            return isFinal;
                        }
                ),

                new RobotAutoPitchCommand(0.65,claw,robotState), //this is a slight incline as we move up to avoid the wall
                new WaitCommand(100),
                //start pulling the linkage in as the arm moves up
                new LinkageInCommand(claw, arm, slide, robotState),
                //lift the arm up to almost the height we want, so it can wobble
                new ArmHighAuto5Command(2550, arm),

                new TurretRightUpCloseFastAutoCommand(arm, slide, turret, claw, robotState),
                        new TurretRightUpCloseAutoCommand(arm, slide, turret, claw, robotState),
                new ArmHighAuto5Command(2650, arm), //this is the arm height over the junction

                new ArmHelperOutCommand(arm),
                //give time for the junction tool to come out
                new WaitCommand(250),
                //move the linkage out - extendo to push the tool into the junction
                new LinkageMoveCommand(0.30, claw, arm, slide, robotState), //smaller = longer
                new WaitCommand(100),
                new RobotAutoPitchCommand(0.83, claw, robotState), //0.75 - higher is more aligned to the cone
                //stablisation amount - this adds up
                new WaitCommand(310),
                //drop the cone
                new RobotClawOpen(claw,arm,slide, robotState),

                new WaitCommand(100),
                new ArmHelperInCommand(arm),
                new WaitCommand(200),
                new LinkageInCommand(claw, arm, slide, robotState),


                        new ConditionalCommand(

                                new WaitCommand(1),// ArmClawReadyCommand(arm,turret, robotState),
                                new SequentialCommandGroup(
                                    new TurretRearDownAutoFastLeftCommand(arm, slide, turret, claw, robotState),
                                    new TurretRearDownAutoLeftCommand(arm, slide, turret, claw, robotState),
                                    new ArmHighAuto5Command(armPos, arm)
                                ),
                                () -> {return isFinal;}
                        )

        );




        addRequirements( arm, slide, claw);
    }


}
