package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.RobotClawClose;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAutoCommand;
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
                        new ArmHighAutoCommand(arm),
                        new SlideToConeCommand(slide, arm),
                        new TurretLeftUpCloseAutoCommand(arm, slide, turret, claw, robotState)
                        //new TurretFrontUpAutoCommand(arm, slide, turret,claw, robotState),
                       // new RobotClawOpen(claw, arm, slide,robotState)

        );



        addRequirements( arm, slide, claw);
    }


}
