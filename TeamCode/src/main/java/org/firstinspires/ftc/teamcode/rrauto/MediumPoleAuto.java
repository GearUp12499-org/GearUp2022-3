package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.lib.AutoLogTelemetry;
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;
import org.firstinspires.ftc.teamcode.lib.VirtualTelemetryLog;
import org.firstinspires.ftc.teamcode.lib.jobs.Job;
import org.firstinspires.ftc.teamcode.lib.jobs.JobManager;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

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

    public static final boolean LOGGING = true;

    private JobManager jobManager;

    @Override
    public void main() {
        telemetry.addLine("Starting up...");
        telemetry.update();
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        jobManager = new JobManager();

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

        stopMaybe();
        waitForStart();
        VirtualTelemetryLog log = new VirtualTelemetryLog(10);
        telemetry = new AutoLogTelemetry(telemetry, log);
        jobManager
                .autoLambda(l::closeClaw)
                .andThen(jobManager.delayJob(500))
                .andThen(() -> { /* Ends immediately */
                    try {
                        l.closeClaw();
                        l.verticalLift(2700, this);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                })
                .andThen(straight(0.6, 52)).start();

        // Main event loop
        runJobsUntilDone();
    }

    /**
     * The main job loop.
     */
    void runJobsUntilDone() {
        while (!jobManager.isDone()) {
            stopMaybe();
            jobManager.invokeAll();

            if (!LOGGING) continue;

            List<Integer> finished = new ArrayList<>();
            List<Integer> running = new ArrayList<>();
            List<Integer> waiting = new ArrayList<>();
            for (Map.Entry<Integer, Job> jobPair : jobManager.getJobs().entrySet()) {
                if (jobPair.getValue().isComplete()) {
                    finished.add(jobPair.getKey());
                } else if (jobPair.getValue().isActive()) {
                    running.add(jobPair.getKey());
                } else {
                    waiting.add(jobPair.getKey());
                }
            }

            int maxPerLine = 5;
            int i = 0;
            StringBuilder b = new StringBuilder();
            b.append("  ");
            telemetry.addLine("Finished: " + finished.size());
            for (Integer job : finished) {
                b.append(job).append(" ");
                if (++i % maxPerLine == 0) {
                    telemetry.addLine(b.toString());
                    b = new StringBuilder();
                    b.append("  ");
                }
            }

            b = new StringBuilder();
            b.append("  ");
            i = 0;
            telemetry.addLine("Running: " + running.size());
            for (Integer job : running) {
                b.append(job).append(" ");
                if (++i % maxPerLine == 0) {
                    telemetry.addLine(b.toString());
                    b = new StringBuilder();
                    b.append("  ");
                }
            }
            telemetry.addLine("Waiting: " + waiting.size());
            for (Integer jobidx : running) {
                Job job = jobManager.getJob(jobidx);
                ArrayList<Integer> deps = job.getDependencies();
                if (deps.size() == 0) {
                    telemetry.addLine("  " + jobidx + " not started");
                } else {
                    StringBuilder b2 = new StringBuilder();
                    b2.append("  ").append(jobidx).append(" waiting on ");
                    for (Integer dep : deps) {
                        b2.append(dep).append(" ");
                    }
                    telemetry.addLine(b2.toString());
                }
            }
            telemetry.update();
        }
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

    // JJogger reimplementation
    public Job moveTurretTo(double speed, int position) {
        // Dummy class to support get/set of data in the Job.
        AtomicBoolean data = new AtomicBoolean(false);

        return new Job(
                jobManager,
                () -> { /* onStart */
                    data.set(turret.getCurrentPosition() < position);
                    turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.setTargetPosition(position);
                    if (turret.getCurrentPosition()<position)
                        turret.setPower(speed);
                    else if (turret.getCurrentPosition()>position)
                        turret.setPower(-speed);
                    else {
                        turret.setPower(0);
                    }
                },
                () -> { /* task */
                    if (Math.abs(turret.getCurrentPosition()-position) < 150) {
                        turret.setPower(0.2 * (data.get() ? 1 : -1));
                    }
                },
                () -> { /* completeCondition */
                    return data.get() ? turret.getCurrentPosition() >= position : turret.getCurrentPosition() <= position;
                },
                () -> turret.setPower(0)
        );
    }

    public Job straight(double speed, double distance) {
        // 1700 ec approx. 1 in

        double adjust = 0.05;
        AtomicInteger targetEncoderCounts = new AtomicInteger((int) (distance * 1700.0));

        return new Job(
                jobManager,
                () -> targetEncoderCounts.set((int) (distance * 1700.0) + encoderLeft.getCurrentPosition()), /* onStart */
                () -> { /* task */
                    double realSpeed = speed;
                    if (encoderLeft.getCurrentPosition() / 1700.0 > 41) {
                        realSpeed = 0.2;
                    }
                    if (encoderLeft.getCurrentPosition() > encoderRight.getCurrentPosition()) {
                        frontLeft.setPower(realSpeed-adjust);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed-adjust);
                        rearRight.setPower(realSpeed);
                    } else if (encoderLeft.getCurrentPosition() < encoderRight.getCurrentPosition()) {
                        frontLeft.setPower(realSpeed+adjust);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed+adjust);
                        rearRight.setPower(realSpeed);
                    } else if (encoderLeft.getCurrentPosition() == encoderRight.getCurrentPosition()) {
                        frontLeft.setPower(realSpeed);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed);
                        rearRight.setPower(realSpeed);
                    }
                },
                () -> frontLeft.getCurrentPosition() > targetEncoderCounts.get(), /* completeCondition */
                () -> { /* onComplete */
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    rearLeft.setPower(0);
                    rearRight.setPower(0);
                }
        );
    }
}
