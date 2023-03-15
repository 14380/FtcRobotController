package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightCloseHighCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;

@Autonomous(group = "drive")
public class RightHighStackAuto extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;


    @Override
    public void initialize() {
        drive = new DriveSubsystem(
                new BotBuildersMecanumDrive(hardwareMap), null, telemetry);

        slide = new SlideSubsystem(hardwareMap, telemetry);

        claw = new ClawSubsystem(hardwareMap, telemetry);

        arm = new ArmSubsystem(hardwareMap);

        turret = new TurretSubsystem(hardwareMap, telemetry);

       // vision = new VisionSubsystem(hardwareMap,telemetry);

        rState = new RobotStateSubsytem(hardwareMap);

        //drive.setPoseEstimate(new Pose2d(-35, 0, Math.toRadians(-90)));








        schedule(new WaitUntilCommand(this::isStarted).andThen(

                //This is the start of the cycling

                new Stack5RightCloseHighCommand(760,0.35,arm, slide, claw, turret, rState),
                new Stack5RightCloseHighCommand(755,0.3,arm, slide, claw, turret, rState),
                new Stack5RightCloseHighCommand(750,0.25,arm, slide, claw, turret, rState),
                new Stack5RightCloseHighCommand(745,0.2,arm, slide, claw, turret, rState),
                new Stack5RightCloseHighCommand(720,0.1,arm, slide, claw, turret, rState),

                new WaitCommand(3000)


                ));

    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){
        //vision.disable();
    }

}
