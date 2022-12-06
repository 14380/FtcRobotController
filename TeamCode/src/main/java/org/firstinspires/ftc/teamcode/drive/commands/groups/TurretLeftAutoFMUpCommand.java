package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretAutoFMLeft;
import org.firstinspires.ftc.teamcode.drive.commands.TurretAutoLeft;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretLeftAutoFMUpCommand extends SequentialCommandGroup {

    public TurretLeftAutoFMUpCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret)
    {


        addCommands(    new ArmHighCommand(arm),
                new WaitCommand(150),
                new ParallelCommandGroup(

                        new SlideUpTopAutoCommand(slide),
                        new TurretAutoFMLeft(turret)
                )

        );



        addRequirements( arm, slide, turret);
    }


}
