package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.AutoClawGrabStartRightHighCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.RobotAutoSlideGraspCommand;
import org.firstinspires.ftc.teamcode.drive.commands.autogroups.TurretRearDownAutoCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class DriveTestAuto extends AutoOpBase {

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

        drive.setPoseEstimate(new Pose2d(-64, 26, Math.toRadians(180)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-64, 26, Math.toRadians(180)))

                //.setConstraints(BotBuildersMecanumDrive.getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                //        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
               // .setReversed(true)
                .lineToSplineHeading(new Pose2d(-22, 26, Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(-4, 26, Math.toRadians(90)))
                .lineToSplineHeading(new Pose2d(-2,5, Math.toRadians(90)))
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(0, 54, Math.toRadians(-90)))
                //.turn(Math.toRadians(-90))
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
                          new ClawGrabCommand(arm,slide,claw,turret,rState),
                        parkFollower //,
                      //  traj2Follower
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
