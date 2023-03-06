package org.firstinspires.ftc.teamcode.drive.commands.autogroups;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoTurretModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHighPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.TurretAutoLeftClose;
import org.firstinspires.ftc.teamcode.drive.commands.auto.TurretAutoRightClose;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

public class TurretRightUpCloseAutoCommand extends SequentialCommandGroup {

    public TurretRightUpCloseAutoCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret,
            ClawSubsystem claw,
            RobotStateSubsytem rState)
    {


        addCommands(
                new AutoSlideModeCommand(rState),
                new AutoTurretModeCommand(rState),
                new ArmHighAuto5Command(1050,arm),
                new RobotClawHighPitchCommand(claw,rState),
                new TurretAutoRightClose(turret)
              /*  new ParallelCommandGroup(
                        new ConditionalCommand(
                                new SlideUpTopCommand(slide),
                                new SlideMid1StackCommand(slide),
                                new BooleanSupplier() {
                                    @Override
                                    public boolean getAsBoolean() {
                                        return slide.IsSlideAtTop() || slide.IsSlideAtBottom() || slide.IsAtGrasp();
                                    }
                                }),
                                new TurretAutoLeft(turret)
                )*/

        );

        addRequirements( arm, slide, turret);
    }


}
