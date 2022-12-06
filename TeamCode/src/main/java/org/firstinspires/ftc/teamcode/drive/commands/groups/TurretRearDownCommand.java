package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretFrontOut;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRear;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRearDownCommand extends SequentialCommandGroup {

    public TurretRearDownCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret,
            RobotStateSubsytem rState)
    {


        addCommands(
                        new AutoSlideModeCommand(rState),
                        new ParallelCommandGroup(
                                new TurretFrontOut(turret),
                                new WaitCommand(350),
                                new SlideToConeCommand(slide, arm)

                        )//,
                     //   new WaitCommand(350),
                       // new ArmClawReadyCommand(arm)

        );

        addRequirements( arm, slide, turret);
    }


}
