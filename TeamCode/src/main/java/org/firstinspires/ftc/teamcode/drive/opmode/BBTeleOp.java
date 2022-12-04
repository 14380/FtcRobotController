package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmStackMid1Command;
import org.firstinspires.ftc.teamcode.drive.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideMid1StackCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawReturnCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretLeftUpCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRearDownCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRearOutTopCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRightUpCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;

@Config
@TeleOp(group = "drive")
public class BBTeleOp extends CommandOpMode {

    private DriveCommand driveCommand;
    private GamepadEx gp1;
    private BotBuildersMecanumDrive mecDrive;
    private RobotStateSubsytem rState;
    private ArmSubsystem arm;

    @Override
    public void initialize() {

        mecDrive = new BotBuildersMecanumDrive(hardwareMap);

        gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad1);

        DriveSubsystem driveSystem = new DriveSubsystem(
               mecDrive, gp1, telemetry);

        ClawSubsystem claw = new ClawSubsystem(
                hardwareMap,
                telemetry
        );

        SlideSubsystem slide = new SlideSubsystem(
                hardwareMap,
                telemetry
        );

        TurretSubsystem turret = new TurretSubsystem(
                hardwareMap,
                telemetry
        );

        arm = new ArmSubsystem(
                hardwareMap,
                telemetry
        );

        rState = new RobotStateSubsytem();

        driveCommand = new DriveCommand(
                driveSystem, () -> -gp1.getLeftY(),
                gp1::getLeftX, gp1::getRightX
        );

        schedule(driveCommand);

        //realign IMU
        /*gp1.getGamepadButton(GamepadKeys.Button.X).and(
                new GamepadButton(gp1, GamepadKeys.Button.Y)
        ).whenActive(
                new InstantCommand(driveSystem::Realign)
        );*/
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(driveSystem::Realign)
        );

        gp1.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(
                new ClawGrabCommand(arm, slide,claw, rState),
                new ClawReturnCommand(arm, slide, claw, rState)

        );

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ArmClawReadyCommand(arm, turret, rState));

        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new ArmMidCommand(arm));


        gp1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(claw::Report));

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new TurretRearOutTopCommand(arm, slide, turret)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new TurretRearDownCommand(arm, slide, turret)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new TurretLeftUpCommand(arm, slide, turret)
        );
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new TurretRightUpCommand(arm, slide, turret)
        );

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SlideToMidCommand(slide)
        );

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whenActive(new ArmStackMid1Command(arm, rState));


        // update telemetry every loop
        schedule(new RunCommand(telemetry::update));


        register(claw, driveSystem, slide, turret, arm);
    }



}


