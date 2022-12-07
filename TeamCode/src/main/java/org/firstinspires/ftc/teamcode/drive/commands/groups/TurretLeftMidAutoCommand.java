package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHighAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretLeft;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRight;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretLeftMidAutoCommand extends SequentialCommandGroup {

    public TurretLeftMidAutoCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret)
    {


        addCommands(    new ArmHighAutoCommand(arm),
                        new WaitCommand(150),
                new ParallelCommandGroup(

                        new SlideToMidAutoCommand(slide),
                        new TurretLeft(turret)
                )

        );

        addRequirements( arm, slide, turret);
    }


}
