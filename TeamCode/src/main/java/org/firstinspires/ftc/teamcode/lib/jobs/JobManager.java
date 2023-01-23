package org.firstinspires.ftc.teamcode.lib.jobs;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Supplier;

import java.util.HashMap;

public class JobManager {
    private int next = 0;
    private final HashMap<Integer, Job> jobs = new HashMap<>();

    public JobManager() {
    }

    public int addJob(Job job) {
        jobs.put(next, job);
        return next++;
    }

    public void invokeAll() {
        for (Job job : jobs.values()) {
            job.tick();
        }
    }

    public void gc() {
        for (Job job : jobs.values()) {
            if (job.isComplete()) {
                jobs.remove(job.id);
            }
        }
    }

    public Job getJob(int id) {
        return jobs.get(id);
    }

    /**
     * Job that finishes when a condition is true, and does nothing otherwise.
     * @param condition Condition to check.
     * @return Job that finishes when the condition is true.
     */
    public Job predicateJob(Supplier<Boolean> condition) {
        return new Job(this, null, null, condition, null, null);
    }

    /**
     * Job that finishes after a certain amount of time.
     * @param millis Time in milliseconds.
     * @return Job that finishes after the specified time.
     */
    public Job delayJob(long millis) {
        ElapsedTime timer = new ElapsedTime();
        return predicateJob(() -> timer.milliseconds() >= millis);
    }
}