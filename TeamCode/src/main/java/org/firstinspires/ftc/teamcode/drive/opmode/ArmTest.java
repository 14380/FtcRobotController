package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;

@Config
@TeleOp(group = "drive")
//@Disabled
public class ArmTest extends OpMode {

    private PIDController controller;
    public static double p=0.00089, i=0.0019, d=0.00000001;
    public static double f = 0.109;

    public static int target = 0;

    private final double ticks_in_degree = 4096 / 180;

    private DcMotorEx armMotor;
    private Servo pitchServo;

    private ServoImplEx linkageServo;
    private ServoImplEx linkageServo2;
    private VoltageSensor voltageSensor;
    private double voltage;

    private ElapsedTime voltageTimer;

    @Override
    public void init(){

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        pitchServo = hardwareMap.get(Servo.class, "clawVertServo");
        linkageServo = hardwareMap.get(ServoImplEx.class, "linkageServo");
        linkageServo2 = hardwareMap.get(ServoImplEx.class, "linkage2Servo");

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.voltage = voltageSensor.getVoltage();
        this.voltageTimer = new ElapsedTime();
        this.voltageTimer.reset();

        linkageServo.setPwmRange(new PwmControl.PwmRange(505, 2495));

        linkageServo2.setDirection(Servo.Direction.REVERSE);
        linkageServo2.setPwmRange(new PwmControl.PwmRange(505, 2495));

        pitchServo.setPosition(0.55);
        linkageServo.setPosition(1);

        linkageServo2.setPosition(1);
    }

    @Override
    public void loop()
    {
        controller.setPID(p,i,d);
        int armPos = armMotor.getCurrentPosition();

        if(this.voltageTimer.seconds() > 5){
            this.voltage = voltageSensor.getVoltage();
            this.voltageTimer.reset();
        }

        double pid = controller.calculate(armPos, target);
        telemetry.addData("pid", pid);
        telemetry.addData("Voltage Add", (voltage / 12));
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double power = pid + ff / voltage * 12.0;
        armMotor.setPower(power);
        telemetry.addData("Pos", armMotor.getCurrentPosition());
        telemetry.addData("Target", target);
        telemetry.update();
    }
}
