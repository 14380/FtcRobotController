package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;

// No turrets on this.
public class SlideUpArmOutTopCommand extends SequentialCommandGroup {

    public SlideUpArmOutTopCommand(
            ArmSubsystem arm,
            ClawSubsystem claw,
            SlideSubsystem slide)
    {


        addCommands(    new ArmMidCommand(arm, claw),
                        new WaitCommand(100),
                        new SlideUpTopCommand(slide)
        );

        addRequirements( arm, slide);
    }


}
