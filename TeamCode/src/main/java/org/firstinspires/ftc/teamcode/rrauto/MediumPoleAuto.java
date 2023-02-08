package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.IOControl;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.lib.AutoLogTelemetry;
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;
import org.firstinspires.ftc.teamcode.lib.MatchCounter;
import org.firstinspires.ftc.teamcode.lib.Supplier;
import org.firstinspires.ftc.teamcode.lib.VirtualTelemetryLog;
import org.firstinspires.ftc.teamcode.lib.jobs.Job;
import org.firstinspires.ftc.teamcode.lib.jobs.JobManager;
import org.firstinspires.ftc.teamcode.lib.jobs.ProfileFile;
import org.firstinspires.ftc.teamcode.lib.jobs.ResultJob;
import org.firstinspires.ftc.teamcode.lib.jobs.ResultJobFactory;
import org.firstinspires.ftc.teamcode.lib.jobs.ToggleableJob;
import org.firstinspires.ftc.teamcode.lib.jobs.ToggleableJobFactory;
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
        MatchCounter.newMatch();  // Increment match number, for storing logs and profile data
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
        ToggleableJob lock = lockPosition();
        ResultJob<Integer> poleDetect1 = poleDetect(-1200, () -> Math.abs(turret.getCurrentPosition()) > 1000);

        Job suffix = jobManager
                .fromLambda(l::closeClaw)                   // Close the claw
                .andThen(jobManager.delayJob(500))    // Wait 500ms
                .andThen(() -> l.setVerticalTargetManual(1800))
                .andThenAsync(liftUntilVertStop())          // Keep updating the lift
                .andThen(straight(0.6, 56))   // Move forward to the pole
                .andThen(poleDetect1)
                .andThen(lock::turnOn)                      // Try to stay in the same position
                .jobSequence(this::scoreCone);

        for (int i = 0; i < 3; i++) {
            int finalI = i;
            suffix = suffix.andThenAsync(moveTurretTo(0.8, 700))
                    .andThen(jobManager.delayJob(500))
                    .andThen(() -> {
                        l.setHorizontalTargetManual(825);
                        l.setVerticalTargetManual(1180 - finalI * 150);
                    })
                    .andThenAsync(liftUntilVertStop())
                    .andThen(controlledLowerLift(-0.6, 1180 - i * 150))
                    .andThen(l::closeClaw)
                    .andThen(jobManager.delayJob(300))
                    .andThen(() -> {
                        l.setVerticalTargetManual(1180 - (finalI * 150) + 2050);
                        l.liftVertical1.setPower(1);
                        l.liftVertical2.setPower(1);
                    })
                    .andThenAsync(liftUntilVertStop())  // ensure lift keeps running
                    .andThen(jobManager.delayJob(500))
                    .andThen(() -> {
                        l.liftVertical1.setPower(0);
                        l.liftVertical2.setPower(0);
                        l.setHorizontalTargetManual(0);
                    })
                    .andThen(jobManager.delayJob(300))
                    .andThen(() -> l.setVerticalTargetManual(3500))
                    .andThen(moveTurretTo(0.5, () -> poleDetect1.getResult() + 37))
                    .jobSequence(this::scoreCone);
        }
        suffix = suffix.andThen(lock::turnOff).andThen(() -> l.setVerticalTarget(2));  // Guarantee a clean stop
        suffix.start();  // DFS time ;)

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
        runJobsUntilDone(() -> {
        });
    }

    /**
     * The main job loop.
     */
    @SuppressLint("DefaultLocale")
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

                double percentage = 100.0 * (double) finished.size() / (double) (finished.size() + running.size() + waiting.size());
                telemetry.addLine(String.format("> %.2f%% done", percentage));
                telemetry.addLine("  Running:");
                for (int id : running) {
                    Job job = jobManager.getJob(id);
                    telemetry.addLine(String.format("    %s", job.toString()));
                }
                telemetry.addLine("  Waiting:");
                for (int id : waiting) {
                    Job job = jobManager.getJob(id);
                    telemetry.addLine(String.format("    (%s) %s", job.getDependencies().size() == 0 ? "idle" : job.getDependencies().size(), job));
                }
                telemetry.addLine("  Finished:");
                for (int id : finished) {
                    Job job = jobManager.getJob(id);
                    telemetry.addLine(String.format("    %s", job.toString()));
                }
            }
            telemetry.update();
        }
        ProfileFile profile = new ProfileFile(jobManager.getJobs().values());
        profile.build();
        profile.export("profile_match_" + MatchCounter.getMatchNumber() + ".jobprof");
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
        if (turret != null) {
            turret.setPower(0);
        }
    }

    Job scoreCone(Job before) {
        return before.andThen(() -> l.setVerticalTargetManual(3500))
                .andThen(liftUntilVertStop())
                .andThen(liftHorizontal(304))
                .andThen(jobManager.delayJob(100))
                .andThen(controlledLowerLift(-0.4, 2600)) // Slowly lower...
                .andThen(() -> l.openClaw())
                .andThen(jobManager.delayJob(250))
                .andThen(liftHorizontal(0));

    }

    Job controlledLowerLift(double speed, int target) {
        return jobManager.factory.manager(jobManager)
                .onStart(() -> {
                    l.liftVertical1.setPower(speed);
                    l.liftVertical2.setPower(speed);
                })
                .completeCondition(() ->
                        l.liftVertical1.getCurrentPosition() <= target)
                .onComplete(() -> {
                    l.liftVertical1.setPower(0);
                    l.liftVertical2.setPower(0);
                })
                .build();
    }

    /**
     * Attempt to keep the robot in the same position by adjusting the power of the motors
     * to compensate for unintentional movement.
     * @return Toggleable job that can be started and stopped.
     */
    @SuppressLint("DefaultLocale")
    ToggleableJob lockPosition() {
        AtomicInteger idealXPos = new AtomicInteger(0);
        AtomicInteger idealYPos = new AtomicInteger(0);
        AtomicBoolean thresh = new AtomicBoolean(false);
        final double RAMP_MAX_SPEED = 0.7;
        final double RAMP = 8000;
        final double Y_MOD = 0;
        final int X_MOD = -1;
        final int THRESHOLD_ON = 2000;
        final int THRESHOLD_OFF = 500;
        final int ABORT = 20000;

        return new ToggleableJobFactory()
                .manager(jobManager)
                .onStart(() -> {
                    idealXPos.set((encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2);
                    idealYPos.set(encoderRear.getCurrentPosition());
                    thresh.set(false);
                })
                .task(rawJob -> {
                    if (!(rawJob instanceof ToggleableJob))
                        throw new ClassCastException("Expected ResultJob, got " + rawJob.getClass().getName() + " instead");
                    ToggleableJob job = (ToggleableJob) rawJob;
                    // Monitor the current position and adjust the power accordingly
                    int currentX = (encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2;
                    int currentY = encoderRear.getCurrentPosition();
                    double xDiff = idealXPos.get() - currentX;
                    xDiff *= X_MOD;
                    double yDiff = idealYPos.get() - currentY;
                    yDiff *= Y_MOD;
                    double magnitude = Math.sqrt(Math.pow(xDiff, 2) + Math.pow(yDiff, 2));
                    double speed = Math.min(magnitude / RAMP, 1) * RAMP_MAX_SPEED;
                    if (magnitude > ABORT) {
                        RobotLog.ee("lockPosition", "Aborting!! The following debugging details may be useful:");
                        RobotLog.ee("lockPosition", String.format("Target     X % 10d and Y % 10d", idealXPos.get(), idealYPos.get()));
                        RobotLog.ee("lockPosition", String.format("Current    X % 10d and Y % 10d", currentX, currentY));
                        RobotLog.ee("lockPosition", String.format("Adj. Delta X % 10.2f and Y % 10.2f", xDiff, yDiff));
                        RobotLog.ee("lockPosition", String.format("Calculated magnitude %.2f", magnitude));
                        double xNorm = xDiff / magnitude;
                        double yNorm = yDiff / magnitude;
                        RobotLog.ee("lockPosition", String.format("Normalized X % 10.2f and Y % 10.2f", xNorm, yNorm));
                        RobotLog.ee("lockPosition", "Motor powers:");
                        RobotLog.ee("lockPosition", String.format("  frontLeft %.2f", (yNorm + xNorm) * speed));
                        RobotLog.ee("lockPosition", String.format("  frontRight %.2f", (yNorm - xNorm) * speed));
                        RobotLog.ee("lockPosition", String.format("  rearLeft %.2f", (yNorm - xNorm) * speed));
                        RobotLog.ee("lockPosition", String.format("  rearRight %.2f", (yNorm + xNorm) * speed));
                        job.turnOff();
                        throw new RuntimeException("This might get out of hand");
                    }
                    if (magnitude > THRESHOLD_ON) {
                        thresh.set(true);
                    }
                    if (magnitude < THRESHOLD_OFF) {
                        thresh.set(false);
                    }
                    telemetry.addData("lock", (thresh.get() ? "active" : "inactive") + " mag %.2f", magnitude);
                    telemetry.addData("lock",  "x %.2f y %.2f", xDiff, yDiff);
                    if (thresh.get()) {
                        // Create a normalized vector
                        double xNorm = xDiff / magnitude;
                        double yNorm = yDiff / magnitude;
                        // Strafe the robot
                        frontLeft.setPower((yNorm + xNorm) * speed);
                        frontRight.setPower((yNorm - xNorm) * speed);
                        rearLeft.setPower((yNorm - xNorm) * speed);
                        rearRight.setPower((yNorm + xNorm) * speed);
                    }
                })
                .onComplete(() -> {
                    // Stop the motors
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    rearLeft.setPower(0);
                    rearRight.setPower(0);
                })
                .build();
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
    Job moveTurretTo(double speed, int target) {
        return moveTurretTo(speed, () -> target);
    }

    // JJogger reimplementation
    Job moveTurretTo(double speed, Supplier<Integer> integerSupplier) {
        // Dummy class to support get/set of data in the Job.
        AtomicBoolean data = new AtomicBoolean(false);

        return jobManager.factory
                .manager(jobManager)
                .onStart(() -> { /* onStart */
                    data.set(turret.getCurrentPosition() < integerSupplier.get());
                    turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    turret.setTargetPosition(integerSupplier.get());
                    if (turret.getCurrentPosition() < integerSupplier.get())
                        turret.setPower(speed);
                    else if (turret.getCurrentPosition() > integerSupplier.get())
                        turret.setPower(-speed);
                    else {
                        turret.setPower(0);
                    }
                })
                .task(() -> { /* task */
                    if (Math.abs(turret.getCurrentPosition() - integerSupplier.get()) < 150) {
                        turret.setPower(0.2 * (data.get() ? 1 : -1));
                    }
                })
                .completeCondition(() -> { /* completeCondition */
                    return data.get() ? turret.getCurrentPosition() >= integerSupplier.get() : turret.getCurrentPosition() <= integerSupplier.get();
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
                        frontLeft.setPower(realSpeed - adjust);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed - adjust);
                        rearRight.setPower(realSpeed);
                    } else if (encoderLeft.getCurrentPosition() < encoderRight.getCurrentPosition()) {
                        frontLeft.setPower(realSpeed + adjust);
                        frontRight.setPower(realSpeed);
                        rearLeft.setPower(realSpeed + adjust);
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
