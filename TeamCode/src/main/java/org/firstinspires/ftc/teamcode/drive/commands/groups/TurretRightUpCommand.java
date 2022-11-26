package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretLeft;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRight;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRightUpCommand extends SequentialCommandGroup {

    public TurretRightUpCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret)
    {


        addCommands(    new ArmHighCommand(arm),
                new ParallelCommandGroup(
                        new SlideUpTopCommand(slide),
                        new TurretRight(turret)
                )

        );

        addRequirements( arm, slide, turret);
    }


}
