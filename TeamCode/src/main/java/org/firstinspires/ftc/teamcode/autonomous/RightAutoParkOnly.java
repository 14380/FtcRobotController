package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.BotBuildersMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="RightAutoParkOnly", group="Auto")

public class RightAutoParkOnly extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;
    int[] ID_TAG_OF_INTEREST = {0, 12, 1}; // ids of tags used

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        BotBuildersMecanumDrive drive = new BotBuildersMecanumDrive(hardwareMap);
        drive.midPointIntake();
        drive.ClawArmIntakeSet();


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

        telemetry.setMsTransmissionInterval(50);

        Pose2d startPose = new Pose2d(6,62, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        TrajectorySequence tsPos1 = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(30)
                .forward(50)
                        //unload the cone
                //.turn(Math.PI / 2)

                .build();

        int parkingPos = 1;

        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();



            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    for(int i = 0; i < 3; i++) { //go through each id in the array
                        if (tag.id == ID_TAG_OF_INTEREST[i]) {
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }
                    if(tag.id == 0){
                        telemetry.addLine("Park in zone 1!");
                        parkingPos = 1;
                    }
                    if(tag.id == 12){
                        telemetry.addLine("Park in zone 2!");
                        parkingPos = 2;
                    }
                    if(tag.id == 1){
                        telemetry.addLine("Park in zone 3!");
                        parkingPos = 3;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);

                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        waitForStart();
        drive.CloseClaw();
        drive.midPointIntake();
        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        //find the extra distance we need to travel to get our parks.
        if(parkingPos == 3) {
            drive.followTrajectorySequence(tsPos1);
            TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .strafeRight(52)
                    .build();
                drive.followTrajectorySequence(forward);
        }else if(parkingPos == 2){

                drive.followTrajectorySequence(tsPos1);
                TrajectorySequence forward = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .strafeRight(25)
                        .build();
                drive.followTrajectorySequence(forward);

        }else{

            drive.followTrajectorySequence(tsPos1);

        }

    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }
}
