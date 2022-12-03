package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmStackMid1Command;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawReturnCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretLeftUpCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRearDownCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRightMidCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRightUpCommand;
import org.firstinspires.ftc.teamcode.drive.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.SlideSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

@Autonomous(group = "drive")
public class LeftStackAuto extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;

    private TrajectorySequenceFollowerCommand parkFollower;
    private TrajectorySequenceFollowerCommand bkFollower;
    private TrajectorySequenceFollowerCommand backToConeFollower;
    private TrajectorySequenceFollowerCommand backToMidFollower;
    private TrajectorySequenceFollowerCommand backToMid2Follower;
    private TrajectorySequenceFollowerCommand secToStackFollower;


    @Override
    public void initialize() {
        drive = new DriveSubsystem(
                new BotBuildersMecanumDrive(hardwareMap), null, telemetry);

        slide = new SlideSubsystem(hardwareMap, telemetry);

        claw = new ClawSubsystem(hardwareMap, telemetry);

        arm = new ArmSubsystem(hardwareMap, telemetry);

        turret = new TurretSubsystem(hardwareMap, telemetry);

        vision = new VisionSubsystem(hardwareMap,telemetry);

        rState = new RobotStateSubsytem();

        drive.setPoseEstimate(new Pose2d(-35, 0, Math.toRadians(-90)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-35, 0, Math.toRadians(-90)))
                /*.back(51,
                        BotBuildersMecanumDrive.getVelocityConstraint(38, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .forward(3)
                .turn(Math.toRadians(90))
                .forward(2)*/
                //.setReversed(true)
                .setConstraints(BotBuildersMecanumDrive.getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-35, 16, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-33, 58, Math.toRadians(0)))
                .forward(2)
                .build();



        TrajectorySequence bk = drive.trajectorySequenceBuilder(traj.end())
                .strafeLeft(1)
                .forward(2)
                .build();

        TrajectorySequence backToCone = drive.trajectorySequenceBuilder(traj.end())
                .strafeRight(3)
                .back(22)
                .build();

        TrajectorySequence toMidConeFromStack = drive.trajectorySequenceBuilder(backToCone.end())
                .forward(24)
                .build();

        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())
                .back(24)
                .build();

        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())
                .back(28,BotBuildersMecanumDrive.getVelocityConstraint(38, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())
                .back(8)
                .build();

        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())
                .forward(15)
                .build();


        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);
        bkFollower = new TrajectorySequenceFollowerCommand(drive, bk);
        backToConeFollower = new TrajectorySequenceFollowerCommand(drive, backToCone);
        backToMidFollower = new TrajectorySequenceFollowerCommand(drive, toMidConeFromStack);
        backToMid2Follower = new TrajectorySequenceFollowerCommand(drive, toMidConeFromStack);
        secToStackFollower = new TrajectorySequenceFollowerCommand(drive, toStack2);

        /*schedule(new WaitUntilCommand(this::isStarted).andThen(
                new ClawGrabCommand(arm, slide, claw, rState),
                parkFollower
                ));*/

        schedule(new WaitUntilCommand(this::isStarted).andThen(
                 new ClawGrabCommand(arm, slide, claw, rState).andThen(
                        new ParallelCommandGroup(
                                new TurretLeftUpCommand(arm, slide, turret),
                                parkFollower

                        )).andThen(
                         new WaitCommand(1250)).andThen(

                         new ClawReturnCommand(arm, slide, claw, rState)).andThen(
                               new WaitCommand(500)).andThen(
                               new TurretRearDownCommand(arm, slide, turret)).andThen(
                               new WaitCommand(500).andThen(
                               new ArmStackMid1Command(arm, rState).andThen(
                               backToConeFollower).andThen(
                                 new WaitCommand(250).andThen(
                                 new ClawGrabCommand(arm, slide,claw, rState).andThen(
                                 new WaitCommand(500).andThen(
                                 new TurretRightMidCommand(arm, slide, turret).andThen(
                                               backToMidFollower.andThen(
                                 new WaitCommand(250).andThen(
                                 new RobotClawOpen(claw, arm, slide, rState).andThen(
                                 new WaitCommand(250).andThen(
                                 new TurretRearDownCommand(arm, slide, turret).andThen(
                                 new WaitCommand(500).andThen(
                                 new ArmStackMid1Command(arm, rState).andThen(
                                secToStackFollower.andThen(
                                 new ClawGrabCommand(arm, slide,claw, rState).andThen(
                                 new WaitCommand(250).andThen(
                                 new TurretRightMidCommand(arm, slide, turret).andThen(
                                 backToMid2Follower.andThen(
                                 new WaitCommand(250).andThen(
                                 new RobotClawOpen(claw, arm, slide, rState).andThen(
                                 new WaitCommand(250).andThen(
                                 new TurretRearDownCommand(arm, slide, turret).andThen(
                                 new WaitCommand(500).andThen(
                                  new ArmClawReadyCommand(arm, turret, rState).andThen(
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
                        ))

                ))))))))))))))))))))))))));

    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){
        vision.disable();
    }

}
