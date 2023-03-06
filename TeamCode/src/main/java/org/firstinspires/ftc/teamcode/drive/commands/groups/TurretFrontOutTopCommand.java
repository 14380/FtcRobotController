package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideMid1StackCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretFrontOut;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretFrontOutTopCommand extends SequentialCommandGroup {

    public TurretFrontOutTopCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            ClawSubsystem claw,
            TurretSubsystem turret)
    {


        addCommands(    new ArmMidCommand(arm, claw),
                        new WaitCommand(100),
                        new ParallelCommandGroup(
                                new ConditionalCommand(
                                        new SlideUpTopCommand(slide),
                                        new SlideMid1StackCommand(slide),
                                        new BooleanSupplier() {
                                            @Override
                                            public boolean getAsBoolean() {
                                                return slide.IsSlideAtTop() || slide.IsSlideAtBottom();
                                            }
                                        }), new TurretFrontOut(turret)
                        )

        );

        addRequirements( arm, slide, turret);
    }


}
