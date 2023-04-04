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

public class Stack5Right4SlowCloseClawGrabCommand extends SequentialCommandGroup {

    public Stack5Right4SlowCloseClawGrabCommand(
            boolean extraDelay,
            int armPos,
            double linkagePos,
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            TurretSubsystem turret,
            RobotStateSubsytem robotState, boolean isFinal)
    {


        addCommands(
                        //This is the start of the pickup
                        new WaitCommand(50),
                        new RobotClawOpen(claw, arm,slide, robotState),
                        new ArmHighAuto5Command(armPos, arm),
                        new WaitCommand(750),// wait for arm, to get ready
                        new RobotAutoPitchCommand(0.70,claw,robotState), //higher value, more parrell ot ground .this angle is the angle the claw comes into the cone
                        new LinkageMoveCommand(linkagePos, claw, arm, slide, robotState), //linkage out, into the cone
                        new WaitCommand(310), //stabilistion amount - remember this is done 5 times .. so it adds up
                        //picking up off the stack
                        new RobotClawClose(claw, arm, slide ),
                        new WaitCommand(270),
                        new RobotAutoPitchCommand(0.65,claw,robotState), //this is a slight incline as we move up to avoid the wall
                        new WaitCommand(100),
                        //start pulling the linkage in as the arm moves up
                        new LinkageInCommand(claw, arm, slide, robotState),
                        //lift the arm up to almost the height we want, so it can wobble
                        new ArmHighAuto5Command(2550, arm),
                        //first turret move is fast, but will cause overshoot and wobble
                        new TurretLeftUpCloseFastAutoCommand(arm, slide, turret, claw, robotState),
                        //make a slower correction to the target position
                        new TurretLeftUpCloseAutoCommand(arm, slide, turret, claw, robotState),
                        //important value below:
                        new ArmHighAuto5Command(2650, arm), //this is the arm height over the junction

                        new ArmHelperOutCommand(arm),
                        //give time for the junction tool to come out
                        new WaitCommand(300),
                        //move the linkage out - extendo to push the tool into the junction
                        new LinkageMoveCommand(0.36, claw, arm, slide, robotState), //smaller number is further
                        new WaitCommand(100),
                        new RobotAutoPitchCommand(0.83, claw, robotState), //0.75 - higher is more aligned to the junction
                        //stablisation amount - this adds up
                        new WaitCommand(320),
                        //drop the cone
                        new RobotClawOpen(claw,arm,slide, robotState),

                        new WaitCommand(120),
                        new ArmHelperInCommand(arm),
                        new WaitCommand(200),
                        new LinkageInCommand(claw, arm, slide, robotState),


                        new ConditionalCommand(

                                new WaitCommand(1), //the last cone is being picked up, we don't want to do anything.
                                new SequentialCommandGroup(
                                        // we return the turret to the font, on all but the last one
                                        // the last one we want to move at the same time as coming down.
                                    new TurretRearDownAutoFastCommand(arm, slide, turret, claw, robotState), //move fast
                                    new TurretRearDownAutoCommand(arm, slide, turret, claw, robotState), //move slow into correct position
                                    new ArmHighAuto5Command(armPos, arm) // make sure the arm is moving to the starting pickup position, ready for the next cone.
                                ),
                                () -> {return isFinal;}
                        )

        );




        addRequirements( arm, slide, claw);
    }


}
