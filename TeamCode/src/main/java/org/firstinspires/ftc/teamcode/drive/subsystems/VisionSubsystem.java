package org.firstinspires.ftc.teamcode.drive.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class VisionSubsystem extends SubsystemBase {

    private final HardwareMap hwmap;
    private final Telemetry tele;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private boolean disabled = false;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int[] ID_TAG_OF_INTEREST = {0, 12, 1}; // ids of tags used

    boolean tagFound = false;
    AprilTagDetection tagOfInterest;

    public enum ConePos{
        NONE,
        ONE,
        TWO,
        THREE
    }

    public VisionSubsystem(HardwareMap hardwareMap, Telemetry telemetry){
        hwmap = hardwareMap;
        tele = telemetry;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

    }

    public void dump(){
        tele.addData("DUMP", "Dumping" + Math.random());
        tele.update();
    }

    public void disable(){
        disabled = true;
    }

    public ConePos getConePosition(){
        if(!tagFound){
            return ConePos.NONE;
        }else{

            if(tagOfInterest.id == 0){
                return ConePos.ONE;
            }else if(tagOfInterest.id == 12){
                return ConePos.TWO;
            }
            else if(tagOfInterest.id == 1){
                return ConePos.THREE;
            }else{
                return ConePos.NONE;
            }

        }
    }

    @Override
    public void periodic() {

        if(disabled) return;

       // tele.addData("Polling", "Now" + Math.random());

        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {


            for (AprilTagDetection tag : currentDetections) {
                for (int i = 0; i < 3; i++) { //go through each id in the array
                    if (tag.id == ID_TAG_OF_INTEREST[i]) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }
        }

        tele.addData("Found:", tagFound);
        if(tagFound) {
            tele.addData("Tag ID:", tagOfInterest.id);
            if(tagOfInterest.id == 0){
                tele.addData("Pos", "1");
            }
            if(tagOfInterest.id == 12){
                tele.addData("Pos", "2");
            }
            if(tagOfInterest.id == 1){
                tele.addData("Pos", "3");
            }
        }
        tele.update();
    }
}
