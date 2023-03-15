package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.commands.ArmClawReadyCommand;
import org.firstinspires.ftc.teamcode.drive.commands.ArmStackMid1Command;
import org.firstinspires.ftc.teamcode.drive.commands.ArmStackMid2Command;
import org.firstinspires.ftc.teamcode.drive.commands.RobotClawOpen;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawReturnCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretLeftAutoUpCommand;
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
public class LeftMidStackAuto extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;

    private TrajectorySequenceFollowerCommand parkFollower;
    private TrajectorySequenceFollowerCommand parkFollower2;
    private TrajectorySequenceFollowerCommand bkFollower;
    private TrajectorySequenceFollowerCommand backToConeFollower;
    private TrajectorySequenceFollowerCommand backToMidFollower;
    private TrajectorySequenceFollowerCommand backToMid2Follower;
    private TrajectorySequenceFollowerCommand secToStackFollower;

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

        drive.setPoseEstimate(new Pose2d(-35, 0, Math.toRadians(-90)));

        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-35, 0, Math.toRadians(-90)))

                .setConstraints(BotBuildersMecanumDrive.getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-35, 16, Math.toRadians(0)))

                .build();

        //this puts us on top of the mid junction
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj.end())

                .setConstraints(BotBuildersMecanumDrive.getVelocityConstraint(44, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .lineToSplineHeading(new Pose2d(-33, 60, Math.toRadians(0)))
                .forward(2)
                .build();



        //first move over to the stack - 1st pickup
        TrajectorySequence backToCone = drive.trajectorySequenceBuilder(traj2.end())
                .strafeRight(3)
                .back(22)
                .build();

        //now move to towards the mid cone - use the arm length to make travel distance smaller
        TrajectorySequence toMidConeFromStack = drive.trajectorySequenceBuilder(backToCone.end())
                .forward(17)
                .build();

        //move back to the stack again for the 2nd cone
        TrajectorySequence toStack2 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())

                .back(18)
                .build();

        //final time towards the stack
        TrajectorySequence toStack3 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())
                .back(18)
                .build();

        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())
                .back(22,BotBuildersMecanumDrive.getVelocityConstraint(38, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        BotBuildersMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();

        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())

                .turn(Math.toRadians(90))
                .strafeRight(4)
                .build();

        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(toMidConeFromStack.end())
                .forward(27)
                .turn(Math.toRadians(90))
                .build();


        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);
        parkFollower2 = new TrajectorySequenceFollowerCommand(drive, traj2);

        backToConeFollower = new TrajectorySequenceFollowerCommand(drive, backToCone);
        backToMidFollower = new TrajectorySequenceFollowerCommand(drive, toMidConeFromStack);
        backToMid2Follower = new TrajectorySequenceFollowerCommand(drive, toMidConeFromStack);
        secToStackFollower = new TrajectorySequenceFollowerCommand(drive, toStack2);
        thirdToStackFollower = new TrajectorySequenceFollowerCommand(drive, toStack3);


        schedule(new WaitUntilCommand(this::isStarted).andThen(
                 new ClawGrabCommand(arm, slide, claw, turret, rState).andThen(
                                parkFollower.andThen(
                                new TurretRightMidAutoCommand(arm, slide, turret).andThen(
                                new WaitCommand(200).andThen(
                                parkFollower2.andThen(


                         new WaitCommand(1100)).andThen(

                         new ClawReturnCommand(arm, slide, claw, rState)).andThen(
                               new WaitCommand(500)).andThen(
                               new TurretRearDownCommand(arm, slide, turret,claw, rState)).andThen(
                               new WaitCommand(500).andThen(
                               new ArmStackMid1Command(arm, rState).andThen(
                               backToConeFollower).andThen(
                                 new WaitCommand(250).andThen(
                                 new ClawGrabCommand(arm, slide,claw, turret, rState).andThen(
                                 new WaitCommand(500).andThen(
                                  backToMidFollower.andThen(
                                       new TurretRightMidAuto2Command(arm, slide, turret).andThen(
                                 new WaitCommand(1450).andThen(
                                 new RobotClawOpen(claw, arm, slide, rState).andThen(
                                 new WaitCommand(350).andThen(
                                 new TurretRearDownCommand(arm, slide, turret,claw, rState).andThen(
                                 new WaitCommand(500).andThen(
                                 new ArmStackMid2Command(arm, rState).andThen(
                                 secToStackFollower.andThen(
                                 new ClawGrabCommand(arm, slide,claw, turret, rState).andThen(
                                 new WaitCommand(250).andThen(
                                 backToMid2Follower.andThen(
                                        new TurretRightMidAuto2Command(arm, slide, turret).andThen(
                                 new WaitCommand(1500).andThen(
                                 new RobotClawOpen(claw, arm, slide, rState).andThen(
                                 new WaitCommand(350).andThen(
                                 new TurretRearDownCommand(arm, slide, turret,claw, rState).andThen(
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


                                 ))))))))))))))))))))))))))))));

    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){
        vision.disable();
    }

}
