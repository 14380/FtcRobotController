package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoTurretModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.TurretAutoRightClose;
import org.firstinspires.ftc.teamcode.drive.commands.auto.TurretAutoRightCloseFast;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRightUpContestedFastAutoCommand extends SequentialCommandGroup {

    public TurretRightUpContestedFastAutoCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret,
            ClawSubsystem claw,
            RobotStateSubsytem rState)
    {


        addCommands(
                new AutoSlideModeCommand(rState),
                new AutoTurretModeCommand(rState),
                new ParallelCommandGroup(
                        new ArmHighAuto5Command(2800, arm),
                        new SequentialCommandGroup(
                            new TurretAutoRightCloseFast(turret),
                            new TurretAutoRightClose(turret)
                        )
                )


        );

        addRequirements( arm, slide, turret);
    }


}