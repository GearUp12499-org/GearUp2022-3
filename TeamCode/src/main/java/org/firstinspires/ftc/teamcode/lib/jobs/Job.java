package org.firstinspires.ftc.teamcode.lib.jobs;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.lib.Consumer;
import org.firstinspires.ftc.teamcode.lib.Function;
import org.firstinspires.ftc.teamcode.lib.NullTools;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a job that can be run in a JobManager.
 */
public class Job {
    public final int id;
    private final JobManager manager;
    private final Consumer<Job> onStart;
    private final Consumer<Job> task;
    private final Function<Job, Boolean> completeCondition;
    private final Consumer<Job> onComplete;
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
     * Create a new job, for running things "concurrently" with other jobs.<br>
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
            @Nullable Consumer<Job> onStart,
            @Nullable Consumer<Job> task,
            @Nullable Function<Job, Boolean> completeCondition,
            @Nullable Consumer<Job> onComplete,
            @Nullable int[] dependencies
    ) {
        this.onStart = NullTools.withDefault(onStart, ignore -> {
        });
        this.task = NullTools.withDefault(task, ignore -> {
        });
        this.completeCondition = NullTools.withDefault(completeCondition, throwaway -> true);
        this.onComplete = NullTools.withDefault(onComplete, ignore -> {
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
        this.onStart.accept(this);
        this.active = true;
    }

    /**
     * Complete appropriate actions based on the state.
     */
    public void tick() {
        if (isActive()) {
            this.task.accept(this);
            if (completeCondition.apply(this)) {
                completeHandler();
            }
        }
    }

    /**
     * (internal) Finish the job, and start any dependent jobs in the process.
     */
    private void completeHandler() {
        this.onComplete.accept(this);
        RobotLog.i("Completed job " + id + ", so checking " + dependentJobs.size() + " dependents");
        for (int job : dependentJobs) {
            boolean x = manager.getJob(job).dependencyJobs.remove(Integer.valueOf(id));
            if (!x) {
                RobotLog.w("remove fail");
            }
            RobotLog.i("Removed " + id + " from dependencies of " + job);

            if (manager.getJob(job).dependencyJobs.size() <= 0) {
                RobotLog.i("Starting " + job);
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
        dependency.addDependent(this);
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
     * Run multiple jobs in parallel.
     * @param continuations Jobs to run in parallel after this one.
     * @return A wrapper job that will complete when all of the passed jobs complete.
     */
    public Job andThen(Job... continuations) {
        List<Integer> newDependencies = new ArrayList<>();
        for (Job continuation : continuations) {
            andThen(continuation);
            newDependencies.add(continuation.id);
        }
        int[] newDependenciesArray = new int[newDependencies.size()];
        for (int i = 0; i < newDependencies.size(); i++) {
            newDependenciesArray[i] = newDependencies.get(i);
        }

        // Wrapper for dependency support
        return new Job(manager, null, null, null, null, newDependenciesArray);
    }

    /**
     * Run a one-shot function after this job.
     * @param oneShotFunc Function to run after this job.
     * @return The new job that will run the function.
     */
    public Job andThen(Runnable oneShotFunc) {
        Job cont = new Job(manager, null, ignore -> oneShotFunc.run(), null, null, null);
        return andThen(cont);
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
            RobotLog.i("Starting job " + id + " (no dependencies)");
            startHandler();
        } else {
            for (int dependency : dependencyJobs) {
                RobotLog.i("Propagate start to " + dependency);
                manager.getJob(dependency).start();
            }
        }
    }

    public ArrayList<Integer> getDependencies() {
        return dependencyJobs;
    }

    public void markComplete() {
        if (isComplete() || !isActive()) {
            return;
        }
        completeHandler();
    }
}
