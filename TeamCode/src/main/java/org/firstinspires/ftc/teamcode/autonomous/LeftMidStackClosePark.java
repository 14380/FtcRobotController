package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperTeleOpOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHomePitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TurretFrontOut;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoArmCollapseCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartLeftMedCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretRearDownAutoArmLeftCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretRearDownAutoCommand;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;

@Autonomous(group = "drive")
public class LeftMidStackClosePark extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;

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
                .lineToSplineHeading(new Pose2d(62, 0, Math.toRadians(93)))
                .lineToSplineHeading(new Pose2d(13, 0, Math.toRadians(93)))
                .lineToSplineHeading(new Pose2d(13, -10.5, Math.toRadians(93)))
                .build();



        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(traj.end())

                .forward(9)
                .strafeRight(29)
                .build();

        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(pos2.end())

                .forward(25)
                .build();


        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(pos2.end())
                .back(22)
                .build();



        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);


        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(


                        new ParallelCommandGroup(
                                new AutoClawGrabStartLeftMedCommand(robot.arm, slide, claw, turret, rState),

                                parkFollower



                                ).andThen(

                                        new InstantCommand(() -> {
                                            PoseStorage.currentPose = new Pose2d(0,0, Math.toRadians(-93));
                                        }),

                                        new ArmHelperTeleOpOutCommand(robot.arm),
                                        new WaitCommand(200),
                                        new LinkageMoveCommand(0.32, claw, robot.arm, slide, rState),

                                        new WaitCommand(240),
                                        new RobotClawOpen(claw,robot.arm,slide, rState),
                                        new WaitCommand(130),
                                        new ArmHelperInCommand(robot.arm),
                                        new WaitCommand(400),
                                        new LinkageInCommand(claw, robot.arm, slide, rState),
                                        new TurretFrontOut(turret)

                                        )

                                )

                                .andThen(

                                        new SelectCommand(
                                                // the first parameter is a map of commands
                                                new HashMap<Object, Command>() {{

                                                    put(VisionSubsystem.ConePos.ONE,
                                                            new SequentialCommandGroup(
                                                                new TrajectorySequenceFollowerCommand(drive, pos2),
                                                                new AutoArmCollapseCommand(robot.arm, slide, claw, turret, rState),
                                                                new TrajectorySequenceFollowerCommand(drive, pos1)

                                                            ));
                                                    put(VisionSubsystem.ConePos.TWO, new TrajectorySequenceFollowerCommand(drive, pos2));
                                                    put(VisionSubsystem.ConePos.NONE, new TrajectorySequenceFollowerCommand(drive, pos2));
                                                    put(VisionSubsystem.ConePos.THREE,
                                                            new SequentialCommandGroup(
                                                                new TrajectorySequenceFollowerCommand(drive, pos2),
                                                                new TrajectorySequenceFollowerCommand(drive, pos3)
                                                            ));
                                                }},
                                                // the selector
                                                vision::getConePosition
                                        ),


                                            new ConditionalCommand(
                                                    new WaitCommand(10),

                                                new SequentialCommandGroup(
                                                        new TurretRearDownAutoArmLeftCommand(robot.arm, slide, turret, claw, rState),
                                                        new TurretRearDownAutoCommand(robot.arm, slide, turret, claw, rState),
                                                        new RobotClawHomePitchCommand(claw, rState),
                                                        new RobotClawOpen(claw, robot.arm,slide, rState),
                                                        new ArmClawReadyAutoCommand(robot.arm, rState)
                                                ),
                                                    () -> {
                                                        return vision.getConePosition() == VisionSubsystem.ConePos.ONE;
                                                    })

                                )


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