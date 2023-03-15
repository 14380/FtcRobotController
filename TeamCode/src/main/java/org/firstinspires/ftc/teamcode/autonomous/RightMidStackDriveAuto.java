package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHighPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawHigherPitchCommand;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightMedCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoRightDriveMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.Stack5RightCloseClawGrabCommand;
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
public class RightMidStackDriveAuto extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;


    private TrajectorySequenceFollowerCommand parkFollower;
    private TrajectorySequenceFollowerCommand traj2Follower;
    private TrajectorySequenceFollowerCommand traj3Follower;


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

        drive.setPoseEstimate(new Pose2d(72, 0, Math.toRadians(0)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(72, 0, Math.toRadians(0)))

                //.setConstraints(BotBuildersMecanumDrive.getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(26, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(26, 4, Math.toRadians(0)))
                .turn(Math.toRadians(-90))
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .setReversed(true)
                //.lineTo(new Vector2d(26, 10))
                .forward(5)
                .build();



        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);
        traj2Follower = new TrajectorySequenceFollowerCommand(drive, traj2);
        traj3Follower = new TrajectorySequenceFollowerCommand(drive, traj3);


        schedule(new WaitUntilCommand(this::isStarted).andThen(


                        new ParallelCommandGroup(
                                new AutoClawGrabStartRightMedCommand(arm, slide, claw, turret, rState),
                                parkFollower).andThen(

                                        new SequentialCommandGroup(
                                                traj2Follower,
                                               // new TurretLeftUpCloseAutoCommand(arm, slide, turret, claw, rState),

                                               // new WaitCommand(200),
                                                new RobotClawHigherPitchCommand(claw,rState),
                                                 new WaitCommand(200),


                                                new RobotClawOpen(claw, arm, slide, rState),

                                                new WaitCommand(250),
                                               // new LinkageInCommand(claw,arm,slide,rState),
                                                new RobotClawHighPitchCommand(claw, rState),
                                                new TurretRearDownAutoCommand(arm, slide, turret, claw, rState),
                                                // new WaitCommand(500),
                                                new ArmHighAuto5Command(800, arm)
                                        ),

                                        traj3Follower,
                                        new AutoRightDriveMidCommand(500,arm, slide,claw, turret, rState)
                                        //This is the start of the cycling
                                       // new Stack5RightCloseClawGrabCommand(1050,0.3, arm, slide, claw, turret, rState),
                                       // new Stack5RightCloseClawGrabCommand(1030,0.3,arm, slide, claw, turret, rState),
                                       // new Stack5RightCloseClawGrabCommand(990,0.25,arm, slide, claw, turret, rState),
                                       // new Stack5RightCloseClawGrabCommand(950,0.2, arm, slide, claw, turret, rState),
                                       // new Stack5RightCloseClawGrabCommand(940,0.1, arm, slide, claw, turret, rState)
                                                )




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
