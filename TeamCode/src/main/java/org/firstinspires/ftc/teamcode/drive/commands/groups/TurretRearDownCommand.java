package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoTurretModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretFrontOut;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRear;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRearDownCommand extends SequentialCommandGroup {

    public TurretRearDownCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret,
            ClawSubsystem claw,
            RobotStateSubsytem rState)
    {


        addCommands(
                        new LinkageInCommand(claw, arm, slide, rState),
                        new AutoSlideModeCommand(rState),
                        new AutoTurretModeCommand(rState),
                        new ParallelCommandGroup(
                                new TurretFrontOut(turret),
                                new WaitCommand(350),
                                new SlideToConeCommand(slide, arm)

                        )//,
                       // new WaitCommand(350),
                       // new ArmClawReadyCommand(arm, turret,rState)

        );

        addRequirements( arm, slide, turret);
    }


}
