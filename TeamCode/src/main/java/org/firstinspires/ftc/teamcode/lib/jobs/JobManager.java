package org.firstinspires.ftc.teamcode.lib.jobs;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.lib.Supplier;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

public class JobManager {
    private int next = 0;
    private final HashMap<Integer, Job> jobs = new HashMap<>();
    private final HashMap<Integer, String> labels = new HashMap<>();
    public final JobFactory factory = new JobFactory();

    /**
     * Capture stack traces for job creation.
     */
    public static final boolean DEBUG_JOB_SOURCES = true;
    /**
     * Whether to persist labels after jobs are complete and gc()'d.
     */
    public static final boolean PERSIST_LABELS = true;


    public JobManager() {
    }

    private static StackTraceElement getCallSource() {
        Thread thread = Thread.currentThread();
        // Get first stack frame that is *not* in this class
        for (StackTraceElement stackElement : thread.getStackTrace()) {
            if (stackElement.getClassName().contains("lib.jobs")) continue;
            return stackElement;
        }
        return new StackTraceElement("unknown", "", "", -1);
    }

    /**
     * INTERNAL USE ONLY!! If you call this method on a job, it will be registered twice.
     * @param job Job to register.
     * @return new ID of the job.
     */
    public int addJob(Job job) {
        jobs.put(next, job);
        StringBuilder label = new StringBuilder();
        label.append(job.getClass().getSimpleName()).append(" #").append(next);
        if (DEBUG_JOB_SOURCES) {
            StackTraceElement source = getCallSource();
            label.append(" @ ").append(source);
            RobotLog.ii("JobManager", "job " + job.toString() + " from " + getCallSource());
        }
        labels.put(next, label.toString());
        return next++;
    }

    /**
     * Run all jobs.
     */
    public void invokeAll() {
        for (Job job : jobs.values()) {
            job.tick();
        }
    }

    /**
     * Remove all jobs that are complete.
     */
    public void gc() {
        for (Job job : new ArrayList<>(jobs.values())) {
            if (job.isComplete()) {
                RobotLog.ii("JobManager", "Clearing: " + labelFor(job.id));
                jobs.remove(job.id);
                if (!PERSIST_LABELS) labels.remove(job.id);
            }
        }
    }

    public String labelFor(int id) {
        return labels.get(id);
    }

    /**
     * Check if all jobs are done.
     * @return True if all jobs are done, false otherwise.
     */
    public boolean isDone() {
        for (Job job : jobs.values()) {
            if (!job.isComplete()) {
                return false;
            }
        }
        return true;
    }

    /**
     * Get a job by ID.
     * @param id ID of the job.
     * @return Job with the specified ID, or null.
     */
    public Job getJob(int id) {
        return jobs.get(id);
    }

    /**
     * Job that finishes after a certain amount of time.
     * @param millis Time in milliseconds.
     * @return Job that finishes after the specified time.
     */
    public Job delayJob(long millis) {
        ElapsedTime timer = new ElapsedTime();
        // Watch out - the job doesn't necessarily happen immediately, so reset in the onStart handler.
        timer.reset();
        Job j = factory.manager(this)
                .onStart(timer::reset)
                .completeCondition(() -> timer.milliseconds() > millis)
                .build();
        RobotLog.ii("JobManager", j.id + " is delay("+millis+")");
        return j;
    }

    public Job fromLambda(Supplier<Boolean> taskAndCondition) {
        return factory.manager(this).completeCondition(taskAndCondition).build();
    }

    public Job fromLambda(Runnable task, Supplier<Boolean> condition) {
        return factory.manager(this).task(task).completeCondition(condition).build();
    }

    public Job fromLambda(Runnable task) {
        return factory.manager(this).task(task).build();
    }

    public Map<Integer, Job> getJobs() {
        return jobs;
    }
}
