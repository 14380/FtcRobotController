package org.firstinspires.ftc.teamcode.drive.commands.groups;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperTeleOpOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmMoveHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoArmHeightModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoSlideModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.AutoTurretModeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHighPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideMid1StackCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideUpTopCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRear;
import org.firstinspires.ftc.teamcode.drive.commands.TurretRight;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

public class TurretRearOutTopCommand extends SequentialCommandGroup {

    public TurretRearOutTopCommand(
            ArmSubsystem arm,
            SlideSubsystem slide,
            TurretSubsystem turret,
            ClawSubsystem claw,
            RobotStateSubsytem rState)
    {


        addCommands(
                        new AutoSlideModeCommand(rState),
                        new AutoTurretModeCommand(rState),
                        new ArmMoveHighCommand(arm), //move the arm really high to avoid hitting the cones
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
                                new SequentialCommandGroup(
                                        new TurretRear(turret), //move turret
                                        new ArmHighCommand(arm), //move the arm to the height above the junction
                                        new ArmHelperTeleOpOutCommand(arm), //helper goes out
                                        new AutoArmHeightModeCommand(RobotStateSubsytem.ArmHeightPosition.UP, rState)) //set the state, the arm is now up
                        ));

        addRequirements( arm, slide, turret);
    }


}
