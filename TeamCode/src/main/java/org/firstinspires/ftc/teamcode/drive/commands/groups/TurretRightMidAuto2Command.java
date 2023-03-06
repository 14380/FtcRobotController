package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHighAuto2Command;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidAuto2Command;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRight2;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRightMidAuto2Command extends SequentialCommandGroup {

    public TurretRightMidAuto2Command(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret)
    {


        addCommands(    new ArmHighAuto2Command(arm),
                        new WaitCommand(150),
                new ParallelCommandGroup(

                        new SlideToMidAuto2Command(slide),
                        new TurretRight2(turret)
                )

        );

        addRequirements( arm, slide, turret);
    }


}
