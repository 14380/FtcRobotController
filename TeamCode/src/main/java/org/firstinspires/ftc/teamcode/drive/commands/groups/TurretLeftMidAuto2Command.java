package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretLeftAuto2;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretLeftMidAuto2Command extends SequentialCommandGroup {

    public TurretLeftMidAuto2Command(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret)
    {


        addCommands(   // new ArmHighAutoCommand(arm),
                        new WaitCommand(150),
                new ParallelCommandGroup(

                        new SlideToMidAutoCommand(slide),
                        new TurretLeftAuto2(turret)
                )

        );

        addRequirements( arm, slide, turret);
    }


}
