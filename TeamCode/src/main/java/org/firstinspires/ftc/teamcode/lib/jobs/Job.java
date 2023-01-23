package org.firstinspires.ftc.teamcode.lib.jobs;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.lib.NullTools;
import org.firstinspires.ftc.teamcode.lib.Supplier;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

/**
 * Represents a job that can be run in a JobManager.
 */
public class Job {
    public final int id;
    private final JobManager manager;
    private final Runnable onStart;
    private final Runnable task;
    private final Supplier<Boolean> completeCondition;
    private final Runnable onComplete;
    /**
     * List of jobs that are waiting for this job to complete.
     */
    private final ArrayList<Integer> dependentJobs = new ArrayList<>();
    /**
     * List of jobs that this job is waiting for.
     */
    private final ArrayList<Integer> dependencyJobs = new ArrayList<>();
    /**
     * Whether the job is complete.
     */
    private boolean complete = false;
    /**
     * Whether the job is active (i.e. running).
     */
    private boolean active = false;

    /**
     * Create a new job, for running things "concurrently" with other jobs.<br/>
     * Notes: "concurrently" is in quotes because the jobs are actually run in a loop, so no actual
     * concurrency is possible. DO NOT BLOCK IN TASKS, as this will cause the entire program
     * to wait.
     *
     * @param mgr               Job manager to add this job to.
     * @param onStart           Runnable to run when the job starts.
     * @param task              Runnable to run every tick.
     * @param completeCondition Supplier to check if the job is complete.
     * @param onComplete        Runnable to run when the job is complete.
     * @param dependencies      Array of job IDs that must complete before this job starts.
     */
    public Job(
            @NotNull JobManager mgr,
            @Nullable Runnable onStart,
            @Nullable Runnable task,
            @Nullable Supplier<Boolean> completeCondition,
            @Nullable Runnable onComplete,
            @Nullable int[] dependencies
    ) {
        this.onStart = NullTools.withDefault(onStart, () -> {
        });
        this.task = NullTools.withDefault(task, () -> {
        });
        this.completeCondition = NullTools.withDefault(completeCondition, () -> true);
        this.onComplete = NullTools.withDefault(onComplete, () -> {
        });
        this.id = mgr.addJob(this);
        this.manager = mgr;
        if (dependencies != null && dependencies.length > 0) {
            dependencyJobs.clear();
            for (int dependency : dependencies) {
                dependencyJobs.add(dependency);
                mgr.getJob(dependency).addDependent(this);
            }
        }
    }

    /**
     * (internal) Start the job.
     */
    private void startHandler() {
        this.onStart.run();
        this.active = true;
    }

    /**
     * Complete appropriate actions based on the state.
     */
    public void tick() {
        if (isActive()) {
            this.task.run();
            if (completeCondition.get()) {
                completeHandler();
            }
        }
    }

    /**
     * (internal) Finish the job, and start any dependent jobs in the process.
     */
    private void completeHandler() {
        this.onComplete.run();
        for (int job : dependentJobs) {
            manager.getJob(job).dependencyJobs.remove(Integer.valueOf(job));
            if (manager.getJob(job).dependencyJobs.size() <= 0) {
                manager.getJob(job).startHandler();
            }
        }
        this.complete = true;
        this.active = false;
    }

    public void addDependent(Job dependent) {
        dependentJobs.add(dependent.id);
    }

    public void addDependency(Job dependency) {
        if (dependency.hasDependent(id)) {
            return; // already a dependency
        }
        dependencyJobs.add(dependency.id);
        dependency.addDependency(this);
    }

    private boolean hasDependent(int id) {
        return dependentJobs.contains(id);
    }

    public boolean isComplete() {
        return complete;
    }

    public boolean isActive() {
        return active;
    }

    /**
     * Configure the passed job so that it runs after this one.
     * Under the hood, attaches this job as a dependency of the passed job.
     * <b>This will not work if the passed job has already started.</b>
     *
     * @param continuation Job to run after this one.
     * @return The continuation job, for chaining.
     */
    public Job andThen(Job continuation) {
        continuation.addDependency(this);
        return continuation;
    }

    /**
     * If this job is already active or complete, abort.
     * Otherwise, start this job's dependencies, if there are any, or else start this job.
     */
    public void start() {
        if (isActive() || isComplete()) {
            return;
        }
        if (dependencyJobs.size() <= 0) {
            startHandler();
        } else {
            for (int dependency : dependentJobs) {
                manager.getJob(dependency).start();
            }
        }
    }
}
