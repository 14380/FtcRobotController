package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Config
public class ArmSubsystem extends SubsystemBase {

    private DcMotorEx armMotor;
    private VoltageSensor voltageSensor;
    private PIDController controller;
    private ElapsedTime voltageTimer;
    private Servo helperServo;

    private double voltage;
    public static double
            p=0.00099,
            i=0.0019,
            d=0.00000001;
    public static double f = 0.109;

    public static int target = 0;

    private final double ticks_in_degree = 4096 / 180;


    public ArmSubsystem(HardwareMap map)
    {

        armMotor = map.get(DcMotorEx.class, "armMotor");

        helperServo = map.get(Servo.class, "helperServo");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        voltageSensor = map.voltageSensor.iterator().next();
        controller = new PIDController(p,i,d);
        this.voltage = voltageSensor.getVoltage();
        this.voltageTimer = new ElapsedTime();
        this.voltageTimer.reset();
    }

    public void loop()
    {

        if(this.voltageTimer.seconds() > 5){
            this.voltage = voltageSensor.getVoltage();
            this.voltageTimer.reset();
        }
        controller.setPID(p,i,d);
        int armPos = armMotor.getCurrentPosition();

        double pid = controller.calculate(armPos, target);// / (this.voltage * 12);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff / voltage * 12.0;
        armMotor.setPower(power);

    }

    public void Top(){

        target = 2300;

    }

    public void HelperIn(){
        helperServo.setPosition(1);
    }
    public void HelperOut(){
        helperServo.setPosition(0);
    }

    public void TopAuto(){

    }

    public void SuperTopAuto(){

    }

    public void RunToHitTurret(){

    }
    public void ResetEncoder(){
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void Mid(){
        /*armMotor.setTargetPosition(980);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(0.9);*/
        target = 1700;
    }

    public void MidStack5(int position){

        target = position;

    }

    //the stack
    public void MidStack1(){

    }

    public void MidStack2(){


    }

    public boolean isAtTop(){
        return  armMotor.getCurrentPosition()   > 2050;
    }

    public boolean isAtTopAuto(){
        return  armMotor.getCurrentPosition()  > 1450;
    }

    public boolean isAtMid()
    {
        // Return when the mid point is located

        return armMotor.getCurrentPosition()   > 1500 && armMotor.getCurrentPosition() < 1800;
    }

    public boolean isAtMid5(int position)
    {
        // Return when the mid point is located
        double percentOfMovement = Math.abs(position) * 0.20;

        return armMotor.getCurrentPosition()  > (position - percentOfMovement) && armMotor.getCurrentPosition() < (position + percentOfMovement);
    }

    public boolean isAtMidStack1()
    {
        // Return when the mid point is located

        return true; //leftServo.getPosition() > 0.3 && rightServo.getPosition() > 0.3 && rightServo.getPosition() < 0.45 && leftServo.getPosition() < 0.45;
    }

    public boolean isAtMidStack2()
    {
        // Return when the mid point is located

        return true; //leftServo.getPosition() > 0.3 && rightServo.getPosition() > 0.3 && rightServo.getPosition() < 0.5 && leftServo.getPosition() < 0.5;
    }

    public boolean isAtCone(){
        // Return true when the arm is at the bottom.
        return armMotor.getCurrentPosition() < 120;
    }


    public void ReadyForCone(){

        target = 50;
        //always make sure the helper is in when at the cone
        HelperIn();
    }
    public void SlowDown(){


    }

    public int getPosition(){
        return armMotor.getCurrentPosition();
    }

    public int getTarget() {return target;}

}

