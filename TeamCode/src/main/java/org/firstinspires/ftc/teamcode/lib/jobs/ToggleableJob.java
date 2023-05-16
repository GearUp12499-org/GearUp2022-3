package org.firstinspires.ftc.teamcode.lib.jobs;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.lib.Consumer;

public class ToggleableJob extends Job {
    /**
     * Create a new job, for running things "concurrently" with other jobs.<br>
     * This Job can be turned on and off with the turnOn() and turnOff() methods when needed.
     * This Job can never finish by itself, the turnOff() method must be called for it to stop.<br><br>
     * Notes: "concurrently" is in quotes because the jobs are actually run in a loop, so no actual
     * concurrency is possible. DO NOT BLOCK IN TASKS, as this will cause the entire program
     * to wait.
     *
     * @param mgr        Job manager to add this job to.
     * @param onStart    Runnable to run when the job starts.
     * @param task       Runnable to run every tick.
     * @param onComplete Runnable to run when the job is complete.
     */
    public ToggleableJob(@NonNull JobManager mgr, @Nullable Consumer<Job> onStart, @Nullable Consumer<Job> task, @Nullable Consumer<Job> onComplete) {
        super(mgr, onStart, task, ignore -> false, onComplete, new int[]{});
    }

    private boolean enabled = false;

    public void turnOn() {
        enabled = true;
        bumpState(); // Side effects only
    }

    public void turnOff() {
        enabled = false;
        bumpState(); // Side effects only
    }

    public void bumpState() {
        if (enabled && !super.isActive()) {
            RobotLog.ii("ToggleableJob", "turning on " + this);
            reset();
            startHandler();
        } else if (!enabled && super.isActive()) {
            RobotLog.ii("ToggleableJob", "turning off " + this);
            completeHandler();
        }
    }

    @Override
    public boolean isActive() {
        // Ensure actual state matches
        return enabled;
    }
}
