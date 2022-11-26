package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.commands.ArmMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;


//No turrets on this..

public class SlideUpArmOutMidCommand extends SequentialCommandGroup {

    public SlideUpArmOutMidCommand(
            ArmSubsystem arm,
            SlideSubsystem slide)
    {


        addCommands(    new ArmMidCommand(arm),
                        new SlideUpTopCommand(slide)
        );

        addRequirements( arm, slide);
    }


}
