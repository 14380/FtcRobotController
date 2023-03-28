package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmHelperOutCommand;
import org.firstinspires.ftc.teamcode.drive.commands.LinkageInCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotAutoPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.SlideToMidAuto2Command;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.auto.LinkageMoveCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightMedCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightCloseClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightCloseHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretLeftUpFirstCloseAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretRearDownAutoCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class RightHighStackAuto extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;

    private BotBuildersMecanumDrive robot;


    private TrajectorySequenceFollowerCommand toHighJunctionFollower;



    private TrajectorySequenceFollowerCommand toJuncFollower;
    private TrajectorySequenceFollowerCommand toJuncFollower1;
    private TrajectorySequenceFollowerCommand toJuncFollower2;
    private TrajectorySequenceFollowerCommand toJuncFollower3;

    private TrajectorySequenceFollowerCommand toParkFollower;
    private TrajectorySequenceFollowerCommand toParkFollower1;
    private TrajectorySequenceFollowerCommand toParkFollower2;
    private TrajectorySequenceFollowerCommand toParkFollower3;



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

        TrajectorySequence toHighJunc = drive.trajectorySequenceBuilder(new Pose2d(72, 0, Math.toRadians(0)))

                .setReversed(true)
                .lineToSplineHeading(new Pose2d(62, 0, Math.toRadians(-93)))
                .lineToSplineHeading(new Pose2d(13, 0, Math.toRadians(-93)))
                .lineToSplineHeading(new Pose2d(10, -12.5, Math.toRadians(-93)))
                .build();

        TrajectorySequence toParkSpot = drive.trajectorySequenceBuilder(toHighJunc.end())

                .setReversed(true)

                .lineToSplineHeading(new Pose2d(10, 10, Math.toRadians(-93)))
                .build();

        TrajectorySequence toJuncSpot = drive.trajectorySequenceBuilder(toParkSpot.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(10, -12.5, Math.toRadians(-93)))
                .build();


        toHighJunctionFollower = new TrajectorySequenceFollowerCommand(drive, toHighJunc);

        toParkFollower = new TrajectorySequenceFollowerCommand(drive, toParkSpot );
        toParkFollower1 = new TrajectorySequenceFollowerCommand(drive, toParkSpot);
        toParkFollower2 = new TrajectorySequenceFollowerCommand(drive, toParkSpot);
        toParkFollower3 = new TrajectorySequenceFollowerCommand(drive, toParkSpot);

        toJuncFollower = new TrajectorySequenceFollowerCommand(drive, toJuncSpot);
        toJuncFollower1 = new TrajectorySequenceFollowerCommand(drive, toJuncSpot);
        toJuncFollower2 = new TrajectorySequenceFollowerCommand(drive, toJuncSpot);
        toJuncFollower3 = new TrajectorySequenceFollowerCommand(drive, toJuncSpot);



        CommandScheduler.getInstance().schedule(
                new WaitUntilCommand(this::isStarted).andThen(

                new ParallelCommandGroup(
                        new AutoClawGrabStartRightHighCommand(robot.arm, slide, claw, turret, rState),

                        toHighJunctionFollower



                ).andThen(
                        new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new ArmHelperOutCommand(robot.arm),
                                    new WaitCommand(250),
                                    new RobotClawOpen(claw,robot.arm,slide, rState),
                                    new WaitCommand(150),
                                    new ArmHelperInCommand(robot.arm),
                                    new LinkageInCommand(claw, robot.arm, slide, rState),
                                    new TurretRearDownAutoCommand(robot.arm, slide, turret, claw, rState),
                                    new ArmHighAuto5Command(1400, robot.arm),
                                    new WaitCommand(100)
                            ),
                                toParkFollower
                        )

                .andThen(
                //This is the start of the cycling
                        new SequentialCommandGroup(
                            new Stack5RightCloseHighCommand(1400,0.55, robot.arm, slide, claw, turret, rState, false, toJuncFollower),
                            new TurretLeftUpFirstCloseAutoCommand(robot.arm, slide, turret, claw, rState),//,
                            new SlideToMidAuto2Command(slide),
                            new RobotAutoPitchCommand(0.8,claw,rState),
                            new ArmHelperOutCommand(robot.arm),
                            new LinkageMoveCommand(0.3, claw, robot.arm, slide, rState),

                            new WaitCommand(250),
                            new RobotClawOpen(claw,robot.arm,slide, rState),
                            new WaitCommand(150),
                            new ArmHelperInCommand(robot.arm),
                            new LinkageInCommand(claw, robot.arm, slide, rState),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                            new TurretRearDownAutoCommand(robot.arm, slide, turret, claw, rState),
                                            new ArmHighAuto5Command(1400, robot.arm)
                                        ),
                                toParkFollower1
                                )

                            )
                ).andThen(
                        //cycle 2
                                        new SequentialCommandGroup(
                                                new Stack5RightCloseHighCommand(1400,0.55, robot.arm, slide, claw, turret, rState, false, toJuncFollower1),
                                                new TurretLeftUpFirstCloseAutoCommand(robot.arm, slide, turret, claw, rState),//,
                                                new SlideToMidAuto2Command(slide),
                                                new RobotAutoPitchCommand(0.8,claw,rState),
                                                new LinkageMoveCommand(0.3, claw, robot.arm, slide, rState),
                                                new ArmHelperOutCommand(robot.arm),
                                                new WaitCommand(250),
                                                new RobotClawOpen(claw,robot.arm,slide, rState),
                                                new WaitCommand(150),
                                                new ArmHelperInCommand(robot.arm),
                                                new LinkageInCommand(claw, robot.arm, slide, rState),
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new TurretRearDownAutoCommand(robot.arm, slide, turret, claw, rState),
                                                                new ArmHighAuto5Command(1400, robot.arm)
                                                        ),
                                                        toParkFollower2
                                                )

                                        )

                        ).andThen(
                                //cycle 3

                                        new SequentialCommandGroup(
                                                new Stack5RightCloseHighCommand(1400,0.55, robot.arm, slide, claw, turret, rState, false, toJuncFollower2),
                                                new TurretLeftUpFirstCloseAutoCommand(robot.arm, slide, turret, claw, rState),//,
                                                new SlideToMidAuto2Command(slide),
                                                new RobotAutoPitchCommand(0.8,claw,rState),
                                                new LinkageMoveCommand(0.3, claw, robot.arm, slide, rState),
                                                new ArmHelperOutCommand(robot.arm),
                                                new WaitCommand(250),
                                                new RobotClawOpen(claw,robot.arm,slide, rState),
                                                new WaitCommand(150),
                                                new ArmHelperInCommand(robot.arm),
                                                new LinkageInCommand(claw, robot.arm, slide, rState),
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new TurretRearDownAutoCommand(robot.arm, slide, turret, claw, rState),
                                                                new ArmHighAuto5Command(1700, robot.arm)
                                                        ),
                                                        toParkFollower3
                                                )

                                        )


                                )


                //)


                )));

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
