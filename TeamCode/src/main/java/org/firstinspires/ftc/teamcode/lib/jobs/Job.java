package org.firstinspires.ftc.teamcode.lib.jobs;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.lib.NullTools;
import org.firstinspires.ftc.teamcode.lib.Supplier;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

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
     * Number of dependencies that must complete before this job starts.
     */
    public int numDeps;
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
            numDeps = dependencies.length;
            for (int dependency : dependencies) {
                mgr.getJob(dependency).addDependent(this);
            }
        } else {
            begin(); // Start immediately if there are no dependencies.
        }
    }

    /**
     * Start the job.
     */
    public void begin() {
        this.onStart.run();
        this.active = true;
    }

    /**
     * Complete appropriate actions based on the state.
     */
    public void tick() {
        if (active) {
            this.task.run();
            if (completeCondition.get()) {
                end();
            }
        }
    }

    /**
     * Finish the job, and start any dependent jobs in the process.
     */
    public void end() {
        this.onComplete.run();
        for (int job : dependentJobs) {
            manager.getJob(job).numDeps--;
            if (manager.getJob(job).numDeps <= 0) {
                manager.getJob(job).begin();
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
        dependency.addDependency(this);
        numDeps++;
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
}
