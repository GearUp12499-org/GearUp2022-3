package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.IOControl;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.lib.AutoLogTelemetry;
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;
import org.firstinspires.ftc.teamcode.lib.Supplier;
import org.firstinspires.ftc.teamcode.lib.VirtualTelemetryLog;
import org.firstinspires.ftc.teamcode.lib.jobs.Job;
import org.firstinspires.ftc.teamcode.lib.jobs.JobManager;
import org.firstinspires.ftc.teamcode.lib.jobs.ResultJob;
import org.firstinspires.ftc.teamcode.lib.jobs.ResultJobFactory;
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
    private IOControl io;

    @Override
    public void main() {
        telemetry.addLine("Starting up...");
        telemetry.update();
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        io = new IOControl(hardwareMap);
        jobManager = new JobManager();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        AprilTagDetectionPipeline aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tag_size, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);

        final boolean[] isCameraRunning = {false};
        final int[] cameraLoadAttempts = {1};
        final int warnAfter = 5;
        while (!isCameraRunning[0] && opModeInInit()) {
            stopMaybe();

            final boolean[] cameraFailed = {false};
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                    isCameraRunning[0] = true;
                    telemetry.addLine("Camera is live.");
                    telemetry.update();
                }
                @Override
                public void onError(int errorCode) {
                    cameraFailed[0] = true;
                    cameraLoadAttempts[0]++;
                }
            });

            while (!(cameraFailed[0] || isCameraRunning[0]) && opModeInInit()) {
                stopMaybe();
                telemetry.addLine("Starting up camera: att " + cameraLoadAttempts[0]);
                if (cameraLoadAttempts[0] > warnAfter) {
                    telemetry.addLine("The robot is having trouble connecting");
                    telemetry.addLine("to the camera. Try restarting the OpMode,");
                    telemetry.addLine("or, if that doesn't work, restart the robot.");
                }
                telemetry.update();
            }
        }

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
        telemetry.setMsTransmissionInterval(50);
        VirtualTelemetryLog log = new VirtualTelemetryLog(10);
        telemetry = new AutoLogTelemetry(telemetry, log);

        // MAIN SECTION
        // Can't create this in the job chain because we need to be able to get the result later
        ResultJob<Integer> poleDetect1 = poleDetect(-1200, () -> Math.abs(turret.getCurrentPosition()) > 1000);

        jobManager
                .fromLambda(l::closeClaw)                   // Close the claw
                .andThen(jobManager.delayJob(500))    // Wait 500ms
                .andThen(() -> l.setVerticalTargetManual(1800))
                .andThenAsync(liftUntilVertStop())          // Keep updating the lift
                /*.andThen(straight(0.6, 52))*/             // Move forward to the pole
                .andThen(poleDetect1)
                .jobSequence(this::scoreCone).start();

        // Main event loop
        runJobsUntilDone(() -> telemetry.addLine("== running =="));
        while (opModeIsActive()) {
            telemetry.addLine("Waiting...");
            telemetry.addLine("pole pos " + poleDetect1.getResult());
            telemetry.addLine("current tur pos " + turret.getCurrentPosition());
            telemetry.update();
            l.update();
        }
    }

    /**
     * The main job loop.
     */
    void runJobsUntilDone() {
        runJobsUntilDone(() -> {});
    }

    /**
     * The main job loop.
     */
    void runJobsUntilDone(Runnable extraLogging) {
        while (!jobManager.isDone()) {
            stopMaybe();
            jobManager.invokeAll();
            extraLogging.run();

            if (LOGGING) {
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

                // running, waiting, finished
                telemetry.addLine(String.format("Jobs: %d running, %d waiting, %d finished", running.size(), waiting.size(), finished.size()));
                telemetry.addLine("  Running:");
                for (int id : running) {
                    Job job = jobManager.getJob(id);
                    telemetry.addLine(String.format("    %s", job.toString()));
                }
                telemetry.addLine("  Waiting:");
                for (int id : waiting) {
                    Job job = jobManager.getJob(id);
                    telemetry.addLine(String.format("    (%s) %s", job.getDependencies().size() == 0 ? "idle" : job.getDependencies().size(), job.toString()));
                }
                telemetry.addLine("  Finished:");
                for (int id : finished) {
                    Job job = jobManager.getJob(id);
                    telemetry.addLine(String.format("    %s", job.toString()));
                }

                double percentage = 100.0 * (double)finished.size() / (double)(finished.size() + running.size() + waiting.size());
                telemetry.addLine(String.format("> %.2f%% done", percentage));
            }
            telemetry.update();
        }
        jobManager.gc();  // Clean up any completed jobs, freeing one-shots and such
        RobotLog.i("runJobsUntilDone: all done");
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

    Job scoreCone(Job before) {
        return before.andThen(() -> l.setVerticalTargetManual(3300))
                .andThen(liftUntilVertStop())
                .andThen(liftHorizontal(344))
                .andThen(jobManager.delayJob(100))
                .andThen(jobManager.factory.manager(jobManager)
                        .onStart(() -> {
                            l.liftVertical1.setPower(-0.4);
                            l.liftVertical2.setPower(-0.4);
                        })
                        .completeCondition(() ->
                                l.liftVertical1.getCurrentPosition() <= 2600)
                        .onComplete(() -> {
                            l.liftVertical1.setPower(0);
                            l.liftVertical2.setPower(0);
                        })
                        .build()) // Slowly lower...
                .andThen(() -> l.openClaw())
                .andThen(jobManager.delayJob(250))
                .andThen(liftHorizontal(0));

    }

    // Pole detection; defaultPolePos = -430
    ResultJob<Integer> poleDetect(int defaultPolePos) {
        return poleDetect(defaultPolePos, () -> turret.getCurrentPosition() < -300 && turret.getCurrentPosition() > -500);
    }

    // Pole detection; defaultPolePos = -430
    ResultJob<Integer> poleDetect(int defaultPolePos, Supplier<Boolean> validWhen) {

        return new ResultJobFactory<Integer>()
                .manager(jobManager)
                .onStart(rawJob -> {
                    if (!(rawJob instanceof ResultJob))
                        throw new ClassCastException("Expected ResultJob, got " + rawJob.getClass().getName() + " instead");
                    @SuppressWarnings("unchecked")
                    ResultJob<Integer> job = (ResultJob<Integer>) rawJob;
                    job.setResult(defaultPolePos);
                    turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    turret.setPower(-0.25);
                })
                .task(rawJob -> {
                    if (!(rawJob instanceof ResultJob))
                        throw new ClassCastException("Expected ResultJob, got " + rawJob.getClass().getName() + " instead");
                    @SuppressWarnings("unchecked")
                    ResultJob<Integer> job = (ResultJob<Integer>) rawJob; // This is always true
                    int now = (int) io.distSensorM.getDistance(MM);

                    if (now < 350
                            && validWhen.get()
                    ) {
                        job.setResult(turret.getCurrentPosition());
                        job.markComplete();
                        return;
                    }

                    telemetry.addLine("poleDetect (" + job.id + "):");
                    telemetry.addData("  distance", io.distSensorM.getDistance(CM) + "cm");
                })
                .completeCondition(() -> Math.abs(turret.getCurrentPosition()) > 1300)
                .onComplete(() -> turret.setPower(0))
                .build();
    }

    Job liftHorizontal(int target) {
        return jobManager.factory
                .manager(jobManager)
                .onStart(() -> l.setHorizontalTargetManual(target))
                .task(l::update)
                .completeCondition(l::isSatisfiedHorizontally)
                .build();
    }

    Job liftUntilVertStop() {
        return jobManager.factory
                .manager(jobManager)
                .task(job -> {
                    l.update();
                    telemetry.addLine("liftUntilStopped (" + job.id + "):");
                    telemetry.addLine("  vertical1 " + l.liftVertical1.getPower());
                    telemetry.addLine("  vertical2 " + l.liftVertical2.getPower());
                })
                .completeCondition(() -> !l.inMotion)
                .build();
    }

    // JJogger reimplementation
    Job moveTurretTo(double speed, int position) {
        // Dummy class to support get/set of data in the Job.
        AtomicBoolean data = new AtomicBoolean(false);

        return jobManager.factory
                .manager(jobManager)
                .onStart(() -> { /* onStart */
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
                })
                .task(() -> { /* task */
                    if (Math.abs(turret.getCurrentPosition()-position) < 150) {
                        turret.setPower(0.2 * (data.get() ? 1 : -1));
                    }
                })
                .completeCondition(() -> { /* completeCondition */
                    return data.get() ? turret.getCurrentPosition() >= position : turret.getCurrentPosition() <= position;
                })
                .onComplete(() -> turret.setPower(0))
                .build();
    }

    Job straight(double speed, double distance) {
        // 1700 ec approx. 1 in

        double adjust = 0.05;
        AtomicInteger targetEncoderCounts = new AtomicInteger((int) (distance * 1700.0));

        return jobManager.factory
                .manager(jobManager)
                .onStart(
                    () -> targetEncoderCounts.set((int) (distance * 1700.0) + encoderLeft.getCurrentPosition())
                )
                .task(() -> {
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
                })
                .completeCondition(() -> encoderLeft.getCurrentPosition() > targetEncoderCounts.get())
                .onComplete(() -> { /* onComplete */
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    rearLeft.setPower(0);
                    rearRight.setPower(0);
                })
                .build();
    }
}
