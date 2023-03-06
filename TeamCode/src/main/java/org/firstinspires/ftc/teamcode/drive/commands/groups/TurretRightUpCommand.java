package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoTurretModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHighPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideMid1StackCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRight;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretRightUpCommand extends SequentialCommandGroup {

    public TurretRightUpCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret,
            ClawSubsystem claw,
            RobotStateSubsytem rState)
    {


        addCommands(
                        new AutoSlideModeCommand(rState),
                        new AutoTurretModeCommand(rState),
                        new ArmHighCommand(arm),
                        new RobotClawHighPitchCommand(claw, rState),
                        new WaitCommand(150),
                new ParallelCommandGroup(
                        new ConditionalCommand(
                                new SlideUpTopCommand(slide),
                                new SlideMid1StackCommand(slide),
                                new BooleanSupplier() {
                                    @Override
                                    public boolean getAsBoolean() {
                                        return slide.IsSlideAtTop() || slide.IsSlideAtBottom() || slide.IsAtGrasp();
                                    }
                                }),
                        new TurretRight(turret)
                )

        );

        addRequirements( arm, slide, turret);
    }


}
