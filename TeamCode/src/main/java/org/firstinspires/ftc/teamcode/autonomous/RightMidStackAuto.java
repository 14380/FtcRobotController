package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperTeleOpOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHighPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHigherPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHomePitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToConeCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightMedCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightCloseClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightCloseHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretLeftUpCloseAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretLeftUpCloseFastAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretRearDownAutoCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

@Autonomous(group = "drive")
public class RightMidStackAuto extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    //private ArmSubsystem arm;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;


    private TrajectorySequenceFollowerCommand parkFollower;


    private BotBuildersMecanumDrive robot;

    @Override
    public void initialize() {
        robot = new BotBuildersMecanumDrive(hardwareMap);
        drive = new DriveSubsystem(
                robot, null, telemetry);

        robot.PitchUp();

        robot.LinkageIn();
        robot.arm.MidStack5(0);

        slide = new SlideSubsystem(hardwareMap, telemetry);

        claw = new ClawSubsystem(hardwareMap, telemetry);

        turret = new TurretSubsystem(hardwareMap, telemetry);

        vision = new VisionSubsystem(hardwareMap,telemetry);

        rState = new RobotStateSubsytem(hardwareMap);
        drive.setPoseEstimate(new Pose2d(72, 0, Math.toRadians(0)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(72, 0, Math.toRadians(0)))

                .setReversed(true)
                .lineToSplineHeading(new Pose2d(62, 0, Math.toRadians(-93)))
                .lineToSplineHeading(new Pose2d(13, 0, Math.toRadians(-93)))
                .lineToSplineHeading(new Pose2d(13, 10.5, Math.toRadians(-93)))
                .build();

        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(traj.end())
                .forward(34)
                .build();

        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(traj.end())

                //.turn(Math.toRadians(90))
                .forward(12)
                .build();

        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(traj.end())
                .back(15)
                //.turn(Math.toRadians(90))
                .build();



        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(


                        new ParallelCommandGroup(
                                new AutoClawGrabStartRightMedCommand(robot.arm, slide, claw, turret, rState),

                                parkFollower



                                ).andThen(
                                        new WaitCommand(50),
                                        //new ArmHelperOutCommand(robot.arm),
                                        new ArmHelperTeleOpOutCommand(robot.arm),
                                        //new WaitCommand(250),
                                        new RobotClawOpen(claw,robot.arm,slide, rState),
                                        new ArmHelperInCommand(robot.arm),
                                        //new WaitCommand(100),

                                        new LinkageInCommand(claw, robot.arm, slide, rState),
                                        new TurretRearDownAutoCommand(robot.arm, slide, turret, claw, rState),
                                        new ArmHighAuto5Command(1400, robot.arm)//,
                                       // new WaitCommand(100)

                                )

                                .andThen(



                                        //This is the start of the cycling
                                        new Stack5RightCloseClawGrabCommand(1350,0.53, robot.arm, slide, claw, turret, rState, false),
                                        new Stack5RightCloseClawGrabCommand(1250,0.55, robot.arm, slide, claw, turret, rState, false),
                                        new Stack5RightCloseClawGrabCommand(1190,0.48, robot.arm, slide, claw, turret, rState, false),
                                        new Stack5RightCloseClawGrabCommand(1140,0.48, robot.arm, slide, claw, turret, rState, false),
                                        new Stack5RightCloseClawGrabCommand(1120,0.45, robot.arm, slide, claw, turret, rState, true),

                                        new ParallelCommandGroup(
                                         new SelectCommand(
                                                // the first parameter is a map of commands
                                                new HashMap<Object, Command>() {{
                                                    put(VisionSubsystem.ConePos.NONE, new TrajectorySequenceFollowerCommand(drive, pos1));
                                                    put(VisionSubsystem.ConePos.ONE, new TrajectorySequenceFollowerCommand(drive, pos1));
                                                    put(VisionSubsystem.ConePos.TWO, new TrajectorySequenceFollowerCommand(drive, pos2));
                                                    put(VisionSubsystem.ConePos.THREE, new TrajectorySequenceFollowerCommand(drive, pos3));
                                                }},
                                                // the selector
                                                vision::getConePosition
                                        ),
                                        new RobotClawHomePitchCommand(claw, rState),
                                        new ArmHighAuto5Command(50,robot.arm))

                        ))
    );


    }

    @Override
    public void run(){
        telemetry.addData("Arm Pos", robot.arm.getPosition());
        telemetry.addData("Target", robot.arm.getTarget());
        telemetry.update();
        CommandScheduler.getInstance().run();
        robot.arm.loop();
    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){
        //vision.disable();
    }

}
