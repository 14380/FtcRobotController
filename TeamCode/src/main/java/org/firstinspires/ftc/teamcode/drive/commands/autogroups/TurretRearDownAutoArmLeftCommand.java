package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoTurretModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretFrontOutAuto;
import org.firstinspires.ftc.teamcode.drive.commands.TurretFrontOutAutoLeft;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRearDownAutoArmLeftCommand extends SequentialCommandGroup {

    public TurretRearDownAutoArmLeftCommand(
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
                                new ArmHighAuto5Command(1800, arm),
                                new TurretFrontOutAutoLeft(turret),
                                new SlideToConeCommand(slide, arm)

                        )


        );

        addRequirements( arm, slide, turret);
    }


}
