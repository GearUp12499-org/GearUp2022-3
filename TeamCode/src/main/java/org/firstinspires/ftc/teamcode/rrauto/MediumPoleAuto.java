package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.runtime;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;
import org.firstinspires.ftc.teamcode.lib.StopOpMode;
import org.firstinspires.ftc.teamcode.lib.jobs.Job;
import org.firstinspires.ftc.teamcode.lib.jobs.JobManager;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class MediumPoleAuto extends LinearCleanupOpMode {
    private Lift l = null;

    // AprilTags magic numbers
    public static final double tag_size = 0.166; // metres
    public static final double fx = 578.272;
    public static final double fy = 578.272;
    public static final double cx = 402.145;
    public static final double cy = 221.506;

    public static final int LEFT = 1;
    public static final int CENTER = 2;
    public static final int RIGHT = 3;

    @Override
    public void main() {
        telemetry.addLine("Starting up...");
        telemetry.update();
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        JobManager jobManager = new JobManager();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tag_size, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                telemetry.addLine("Camera is live.");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                throw new RuntimeException("Camera failed to open, code " + errorCode);
            }
        });

        AprilTagDetection tagOfInterest = null;
        int targetLocation = 2;

        while (opModeInInit()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == CENTER || tag.id == RIGHT) {
                        if (tag.id == LEFT) {
                            targetLocation = 1;
                        }
                        if (tag.id == CENTER)
                            targetLocation = 2;
                        if (tag.id == RIGHT)
                            targetLocation = 3;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("See a tag; id is " + tagOfInterest.id + " and our target is " + targetLocation);
                } else {
                    if (tagOfInterest == null) {
                        telemetry.addLine("Don't see a matching tag; no history; target is " + targetLocation);
                    } else {
                        telemetry.addLine("Don't see a matching tag; historical tag id is " + tagOfInterest.id + " and our target is " + targetLocation);
                    }
                }
            } else {
                if (tagOfInterest == null) {
                    telemetry.addLine("Don't see any tag; no history; target is " + targetLocation);
                } else {
                    telemetry.addLine("Don't see any tag; historical tag id is " + tagOfInterest.id + " and our target is " + targetLocation);
                }
            }
            telemetry.update();
            safeSleep(500);
        }
        jobManager.delayJob(1000).andThen(jobManager.delayJob(500)).andThen(jobManager.delayJob(500));
    }

    @Override
    public void cleanup() {
        if (frontLeft != null) {
            frontLeft.setPower(0);
        }
        if (frontRight != null) {
            frontRight.setPower(0);
        }
        if (rearLeft != null) {
            rearLeft.setPower(0);
        }
        if (rearRight != null) {
            rearRight.setPower(0);
        }
        if (l != null) {
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            l.liftHorizontal.setPower(0);
        }
    }
}
