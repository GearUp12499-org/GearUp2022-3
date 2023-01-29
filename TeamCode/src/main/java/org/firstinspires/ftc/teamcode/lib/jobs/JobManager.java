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
    public final JobFactory factory = new JobFactory();

    public JobManager() {
    }

    /**
     * INTERNAL USE ONLY!! If you call this method on a job, it will be registered twice.
     * @param job Job to register.
     * @return new ID of the job.
     */
    public int addJob(Job job) {
        jobs.put(next, job);
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
                RobotLog.i("Clearing job " + job.id);
                jobs.remove(job.id);
            }
        }
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
     * Job that finishes when a condition is true, and does nothing otherwise.
     * @param condition Condition to check.
     * @return Job that finishes when the condition is true.
     */
    public Job predicateJob(Supplier<Boolean> condition) {
        return factory.completeCondition(condition).build();
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

    public Job autoLambda(Supplier<Boolean> taskAndCondition) {
        return factory.completeCondition(taskAndCondition).build();
    }

    public Job autoLambda(Runnable task, Supplier<Boolean> condition) {
        return factory.task(task).completeCondition(condition).build();
    }

    public Job autoLambda(Runnable task) {
        return factory.task(task).build();
    }

    public Map<Integer, Job> getJobs() {
        return jobs;
    }
}
