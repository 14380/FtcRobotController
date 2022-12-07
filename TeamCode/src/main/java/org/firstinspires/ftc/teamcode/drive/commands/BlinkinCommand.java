package org.firstinspires.ftc.teamcode.drive.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.subsystems.RobotStateSubsytem;
import org.firstinspires.ftc.teamcode.drive.subsystems.VisionSubsystem;

public class BlinkinCommand extends CommandBase {

    ElapsedTime timer = new ElapsedTime();
    private RobotStateSubsytem rState;

    public BlinkinCommand(RobotStateSubsytem robot) {
        rState = robot;
        addRequirements(rState);
    }

    @Override
    public void initialize() {

        timer.startTime();
    }

    @Override
    public void execute() {

        double elapsedTime = 120 - timer.seconds();
        rState.setPattern(elapsedTime);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
