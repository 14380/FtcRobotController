package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.commands.TrajectorySequenceFollowerCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawGrabCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.ClawReturnCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretLeftUpCommand;
import org.firstinspires.ftc.teamcode.drive.commands.groups.TurretRearDownCommand;
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
@Disabled
public class LeftSingleCone  extends AutoOpBase {

    private DriveSubsystem drive;
    private SlideSubsystem slide;
    private ClawSubsystem claw;
    private ArmSubsystem arm;
    private TurretSubsystem turret;
    private VisionSubsystem vision;
    private RobotStateSubsytem rState;

    private TrajectorySequenceFollowerCommand parkFollower;
    private TrajectorySequenceFollowerCommand bkFollower;



    @Override
    public void initialize() {
        drive = new DriveSubsystem(
                new BotBuildersMecanumDrive(hardwareMap), null, telemetry);

        slide = new SlideSubsystem(hardwareMap, telemetry);

        claw = new ClawSubsystem(hardwareMap, telemetry);

        arm = new ArmSubsystem(hardwareMap, telemetry);

        turret = new TurretSubsystem(hardwareMap, telemetry);

        vision = new VisionSubsystem(hardwareMap,telemetry);

        rState = new RobotStateSubsytem(hardwareMap);



        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(29)
                .back(48)
                .build();

        TrajectorySequence bk = drive.trajectorySequenceBuilder(traj.end())
                .strafeLeft(4)
                .build();


        TrajectorySequence pos1 = drive.trajectorySequenceBuilder(bk.end())
                .back(2)
                .strafeRight(54)
                .build();

        TrajectorySequence pos2 = drive.trajectorySequenceBuilder(bk.end())
                .back(2)
                .strafeRight(27)
                .build();

        TrajectorySequence pos3 = drive.trajectorySequenceBuilder(bk.end())
                .back(2)
                .strafeRight(4)
                .build();


        parkFollower = new TrajectorySequenceFollowerCommand(drive, traj);
        bkFollower = new TrajectorySequenceFollowerCommand(drive, bk);


        schedule(new WaitUntilCommand(this::isStarted).andThen(
                 new ClawGrabCommand(arm, slide, claw, rState),
                        parkFollower
                       .andThen(
                               new TurretLeftUpCommand(arm, slide, turret,rState).andThen(
                               new WaitCommand(1800)).andThen(
                                       bkFollower ).andThen(
                               new ClawReturnCommand(arm, slide, claw, rState)).andThen(
                               new WaitCommand(500)).andThen(
                               new TurretRearDownCommand(arm, slide, turret,rState)).andThen(
                                       new WaitCommand(500)
                               ),
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
                        )

                )));

    }

    @Override
    public void preInit() {

    }

    @Override
    public void preStart(){
        vision.disable();
    }

}
