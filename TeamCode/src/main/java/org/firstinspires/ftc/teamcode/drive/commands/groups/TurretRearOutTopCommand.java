package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretFrontOut;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRear;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretRearOutTopCommand extends SequentialCommandGroup {

    public TurretRearOutTopCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret)
    {


        addCommands(    new ArmHighCommand(arm),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new ConditionalCommand(
                                        new SlideUpTopCommand(slide),
                                        new SlideUpMidCommand(slide),
                                        new BooleanSupplier() {
                                            @Override
                                            public boolean getAsBoolean() {
                                                return slide.IsSlideAtTop() || slide.IsSlideAtBottom() || slide.IsAtGrasp();
                                            }
                                        }),
                                new TurretRear(turret)
                        ));

        addRequirements( arm, slide, turret);
    }


}
