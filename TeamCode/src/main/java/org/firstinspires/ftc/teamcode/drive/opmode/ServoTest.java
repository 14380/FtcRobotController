package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@TeleOp(group = "drive")
//@Disabled
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        ServoImplEx s1 = hardwareMap.get(ServoImplEx.class, "rightServo");
        ServoImplEx s2 = hardwareMap.get(ServoImplEx.class, "leftServo");

        Servo link = hardwareMap.get(Servo.class, "linkageServo");
        Servo clawV = hardwareMap.get(Servo.class, "clawVertServo");

        s1.setDirection(Servo.Direction.REVERSE);
        s1.setPwmRange(new PwmControl.PwmRange(505, 2495));

        s2.setPwmRange(new PwmControl.PwmRange(505, 2495));


        waitForStart();



        while (!isStopRequested()) {


            if(gamepad1.a){
               s1.setPosition(0.5);
               s2.setPosition(0.5);
            }
            if(gamepad1.b){
                s1.setPosition(0);
                s2.setPosition(0);
            }

            if(gamepad1.x){
                link.setPosition(1);

            }
            if(gamepad1.y){
                link.setPosition(0);

            }

            if(gamepad1.left_bumper){
                clawV.setPosition(0);
            }
            if(gamepad1.right_bumper){
                clawV.setPosition(1);
            }
        }
    }
}
