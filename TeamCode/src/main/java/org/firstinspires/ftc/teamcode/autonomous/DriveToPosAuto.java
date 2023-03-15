package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmStackMid1Command;
import org.firstinspires.ftc.teamcode.drive.commands.ArmStackMid2Command;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.auto.ArmHighAuto5Command;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretRearDownAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawReturnCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRearDownCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRightMidAuto2Command;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRightMidAutoCommand;
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
public class DriveToPosAuto extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;

    private TrajectorySequenceFollowerCommand parkFollower;
    private TrajectorySequenceFollowerCommand traj2Follower;


    private TrajectorySequenceFollowerCommand thirdToStackFollower;


    @Override
    public void initialize() {
        drive = new DriveSubsystem(
                new BotBuildersMecanumDrive(hardwareMap), null, telemetry);

        slide = new SlideSubsystem(hardwareMap, telemetry);

        claw = new ClawSubsystem(hardwareMap, telemetry);

        arm = new ArmSubsystem(hardwareMap);

        turret = new TurretSubsystem(hardwareMap, telemetry);

        vision = new VisionSubsystem(hardwareMap,telemetry);

        rState = new RobotStateSubsytem(hardwareMap);

        drive.setPoseEstimate(new Pose2d(72, 0, Math.toRadians(0)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(72, 0, Math.toRadians(0)))

                //.setConstraints(BotBuildersMecanumDrive.getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(0)))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(22, 5, Math.toRadians(0)))
                .turn(Math.toRadians(-90))
                .build();

        //90 turn -> left facing

        //this puts us on top of the mid junction
      /*  TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())

                .setConstraints(BotBuildersMecanumDrive.getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-33, 60, Math.toRadians(0)))
                //.forward(2)
                .build();*/


        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);
        traj2Follower = new TrajectorySequenceFollowerCommand(drive, traj2);

        schedule(new WaitUntilCommand(this::isStarted)
                .andThen(
                        new ParallelCommandGroup(
                                new AutoClawGrabStartRightHighCommand(arm, slide, claw, turret, rState),
                                parkFollower),
                         new SequentialCommandGroup(
                             new WaitCommand(1500),
                             new ParallelCommandGroup(
                                     traj2Follower,
                                     new TurretRearDownAutoCommand(arm, slide, turret, claw, rState)

                             )
                         )
                )
        );


    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){
        vision.disable();
    }

}
