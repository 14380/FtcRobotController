package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperTeleOpOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHomePitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightContestedAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightContestedDriveAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretRearDownAutoHighCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

@Autonomous(group = "drive")
@Disabled
public class RightHighStackAutoDrive extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;


    private TrajectorySequenceFollowerCommand parkFollower;

    private TrajectorySequenceFollowerCommand moveToCone1;
    private TrajectorySequenceFollowerCommand moveToJunc1;

    private TrajectorySequenceFollowerCommand moveToCone2;
    private TrajectorySequenceFollowerCommand moveToJunc2;

    private TrajectorySequenceFollowerCommand moveToCone3;
    private TrajectorySequenceFollowerCommand moveToJunc3;

    private TrajectorySequenceFollowerCommand moveToCone4;
    private TrajectorySequenceFollowerCommand moveToJunc4;

    private TrajectorySequenceFollowerCommand moveToCone5;
    private TrajectorySequenceFollowerCommand moveToJunc5;


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
                .lineToSplineHeading(new Pose2d(12.5, 0, Math.toRadians(-93)))
                .lineToSplineHeading(new Pose2d(12.5, 10.5, Math.toRadians(-93)))
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

        TrajectorySequence moveToConTraj = drive.trajectorySequenceBuilder(traj.end())
                .back(8)
                .build();

        TrajectorySequence moveToJuncTraj = drive.trajectorySequenceBuilder(moveToConTraj.end())
                .forward(8)
                .build();


        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);

        moveToCone1 = new TrajectorySequenceFollowerCommand(drive, moveToConTraj);
        moveToJunc1 = new TrajectorySequenceFollowerCommand(drive, moveToJuncTraj);

        moveToCone2 = new TrajectorySequenceFollowerCommand(drive, moveToConTraj);
        moveToJunc2 = new TrajectorySequenceFollowerCommand(drive, moveToJuncTraj);

        moveToCone3 = new TrajectorySequenceFollowerCommand(drive, moveToConTraj);
        moveToJunc3 = new TrajectorySequenceFollowerCommand(drive, moveToJuncTraj);

        moveToCone4 = new TrajectorySequenceFollowerCommand(drive, moveToConTraj);
        moveToJunc4 = new TrajectorySequenceFollowerCommand(drive, moveToJuncTraj);

        moveToCone5 = new TrajectorySequenceFollowerCommand(drive, moveToConTraj);
        moveToJunc5 = new TrajectorySequenceFollowerCommand(drive, moveToJuncTraj);


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(


                        new ParallelCommandGroup(
                                new AutoClawGrabStartRightHighCommand(robot.arm, slide, claw, turret, rState),

                                parkFollower



                        ).andThen(
                                        new ArmHelperTeleOpOutCommand(robot.arm),
                                        new RobotClawOpen(claw,robot.arm,slide, rState),
                                        new ArmHelperInCommand(robot.arm),
                                        new LinkageInCommand(claw, robot.arm, slide, rState),
                                        new TurretRearDownAutoHighCommand(robot.arm, slide, turret, claw, rState),
                                        new RobotClawOpen(claw, robot.arm,slide, rState),
                                        new RobotAutoPitchCommand(0.1,claw,rState),
                                        new WaitCommand(200),
                                        new ArmClawReadyAutoCommand(robot.arm, rState)

                                )

                                .andThen(

                                        new Stack5RightContestedDriveAutoCommand(50, 0.53, robot.arm, slide, claw, turret, rState, moveToJunc1, moveToCone1, false),
                                        new Stack5RightContestedDriveAutoCommand(50, 0.53, robot.arm, slide, claw, turret, rState, moveToJunc2, moveToCone2, false),
                                        new Stack5RightContestedDriveAutoCommand(50, 0.53, robot.arm, slide, claw, turret, rState, moveToJunc3, moveToCone3, false),
                                        new Stack5RightContestedDriveAutoCommand(50, 0.53, robot.arm, slide, claw, turret, rState, moveToJunc4, moveToCone4, false),
                                        new Stack5RightContestedDriveAutoCommand(50, 0.53, robot.arm, slide, claw, turret, rState, moveToJunc5, moveToCone5, false),

                                        /*new Stack5RightContestedAutoCommand(1250, 0.55, robot.arm, slide, claw, turret, rState,false),
                                        new Stack5RightContestedAutoCommand(1190, 0.48, robot.arm, slide, claw, turret, rState,false),
                                        new Stack5RightContestedAutoCommand(1140, 0.48, robot.arm, slide, claw, turret, rState,false),
                                        new Stack5RightContestedAutoCommand(1120, 0.45, robot.arm, slide, claw, turret, rState,true),
                                        */

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
