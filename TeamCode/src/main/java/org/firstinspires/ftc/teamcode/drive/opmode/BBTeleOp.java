package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmStackMid1Command;
import org.firstinspires.ftc.teamcode.drive.commands.BlinkinCommand;
import org.firstinspires.ftc.teamcode.drive.commands.DriveCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ManualSlideDownCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ManualSlideHoldCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ManualSlideUpCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ManualTurretHoldCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ManualTurretLeftCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ManualTurretRightCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideMid1StackCommand;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawReturnCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawSlowReturnCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.LinkageOutSlideCommand;
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
    private GamepadEx gp2;
    private BotBuildersMecanumDrive mecDrive;
    private RobotStateSubsytem rState;



    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        mecDrive = new BotBuildersMecanumDrive(hardwareMap);
        mecDrive.PitchUp();
        mecDrive.ReadyForCone();
        mecDrive.LinkageIn();

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

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

       /* arm = new ArmSubsystem(
                hardwareMap,
                telemetry
        );*/

        rState = new RobotStateSubsytem(hardwareMap);

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
        );
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(driveSystem::Realign)
        );*/

        if(gp1.isDown(GamepadKeys.Button.X) && gp1.isDown(GamepadKeys.Button.Y)){
            new InstantCommand(driveSystem::Realign);
        }else {

            gp1.getGamepadButton(GamepadKeys.Button.Y).toggleWhenPressed(

                    new LinkageOutSlideCommand(mecDrive.arm, slide, claw, rState),
                    new LinkageInCommand(claw, mecDrive.arm, slide, rState)

            );

            gp1.getGamepadButton(GamepadKeys.Button.X).toggleWhenPressed(

                    new ClawGrabCommand(mecDrive.arm, slide, claw, turret, rState),
                    new ClawReturnCommand(mecDrive.arm, slide, claw, rState)

            );

        }

        /*gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ClawSlowReturnCommand(mecDrive.arm, turret, claw, rState));*/

        gp1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new ManualTurretRightCommand(slide, mecDrive.arm, turret, rState),
                                new ClawSlowReturnCommand(mecDrive.arm, turret, claw, rState),
                                () -> {
                                    return rState.getArmHightPosition() == RobotStateSubsytem.ArmHeightPosition.UP;
                                })
                ).whenReleased(
                  new ConditionalCommand(
                          new ManualTurretHoldCommand(slide, mecDrive.arm, turret, rState),
                          new ClawSlowReturnCommand(mecDrive.arm, turret, claw, rState),
                          () -> {
                              return rState.getArmHightPosition() == RobotStateSubsytem.ArmHeightPosition.UP;
                          })
                  );


        gp1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(
                        new ConditionalCommand(
                                new ManualTurretLeftCommand(slide, mecDrive.arm, turret, rState),
                                new ArmMidCommand(mecDrive.arm,claw),
                                () -> {
                                    return rState.getArmHightPosition() == RobotStateSubsytem.ArmHeightPosition.UP;
                                })
                ).whenReleased(
                        new ConditionalCommand(
                                new ManualTurretHoldCommand(slide, mecDrive.arm, turret, rState),
                                new ArmMidCommand(mecDrive.arm,claw),
                                () -> {
                                    return rState.getArmHightPosition() == RobotStateSubsytem.ArmHeightPosition.UP;
                                })
                );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new TurretRearOutTopCommand(mecDrive.arm, slide, turret, claw, rState)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new TurretRearDownCommand(mecDrive.arm, slide, turret,claw, rState)
        );

        gp1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new TurretLeftUpCommand(mecDrive.arm, slide, turret,claw, rState)
        );
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new TurretRightUpCommand(mecDrive.arm, slide, turret, claw, rState)
        );

        gp1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SlideToMidCommand(slide)
        );




        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whenActive(new ManualSlideUpCommand(slide, mecDrive.arm, rState));

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
            }
        }).whenActive(new ManualSlideDownCommand(slide, mecDrive.arm, rState));

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5 && gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5;
            }
        }).whenActive(new ManualSlideHoldCommand(slide, mecDrive.arm, rState));

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5;
            }
        }).whenActive(new ManualTurretRightCommand(slide, mecDrive.arm, turret, rState));

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5;
            }
        }).whenActive(new ManualTurretLeftCommand(slide, mecDrive.arm, turret, rState));

        new Trigger(new BooleanSupplier() {
            @Override
            public boolean getAsBoolean() {
                return gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.5 && gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5;
            }
        }).whenActive(new ManualTurretHoldCommand(slide, mecDrive.arm, turret, rState));


        // update telemetry every loop
        //schedule(new RunCommand(telemetry::update));

        rState.setDefaultCommand(new PerpetualCommand(new BlinkinCommand(rState)));

        register(claw, driveSystem, slide, turret, mecDrive.arm);
    }

    @Override
    public void run(){
        //super.run();
        mecDrive.arm.loop();
        CommandScheduler.getInstance().run();
        telemetry.addData("Pos", mecDrive.arm.getPosition());
        telemetry.addData("Target", mecDrive.arm.target);
        telemetry.update();

        if(gp1.isDown(GamepadKeys.Button.X) && gp1.isDown(GamepadKeys.Button.Y)) {
           mecDrive.ReAlignIMU();
        }

    }



}


