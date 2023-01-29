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
        for (Job job : new ArrayList<>(jobs.values())) {
            if (job.isComplete()) {
                RobotLog.i("Clearing job " + job.id);
                jobs.remove(job.id);
            }
        }
    }

    public boolean isDone() {
        for (Job job : jobs.values()) {
            if (!job.isComplete()) {
                return false;
            }
        }
        return true;
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
