package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Concept: Scan Servo", group = "Concept")
@Disabled
public class IntakeTest extends LinearOpMode {


    // Define class members
    CRServo servo1;
    CRServo servo2;

    @Override
    public void runOpMode() {

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        servo1 = hardwareMap.get(CRServo.class, "servo1");
        servo2 = hardwareMap.get(CRServo.class, "servo2");

        servo1.setDirection(DcMotorSimple.Direction.REVERSE);
        // Wait for the start button
        telemetry.addData(">", "Press Start" );
        telemetry.update();
        waitForStart();



        while(opModeIsActive()){

            if(gamepad1.a) {
                servo1.setPower(1);
                servo2.setPower(1);
            }else if(gamepad1.b){
                servo1.setPower(-1);
                servo2.setPower(-1);
            }else{
                servo1.setPower(0);
                servo2.setPower(0);
            }
        }

    }
}
