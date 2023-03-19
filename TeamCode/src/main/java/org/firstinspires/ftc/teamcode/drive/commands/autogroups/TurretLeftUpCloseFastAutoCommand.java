package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoTurretModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.TurretAutoLeftClose;
import org.firstinspires.ftc.teamcode.drive.commands.auto.TurretAutoLeftCloseFast;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretLeftUpCloseFastAutoCommand extends SequentialCommandGroup {

    public TurretLeftUpCloseFastAutoCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret,
            ClawSubsystem claw,
            RobotStateSubsytem rState)
    {


        addCommands(
                new AutoSlideModeCommand(rState),
                new AutoTurretModeCommand(rState),
               // new ArmHighAuto5Command(2200,arm),
               // new RobotClawHighPitchCommand(claw,rState),
                new TurretAutoLeftCloseFast(turret)


        );

        addRequirements( arm, slide, turret);
    }


}
