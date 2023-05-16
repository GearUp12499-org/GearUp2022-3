package org.firstinspires.ftc.teamcode.lib.jobs;

import org.firstinspires.ftc.teamcode.lib.Consumer;
import org.firstinspires.ftc.teamcode.lib.Function;
import org.firstinspires.ftc.teamcode.lib.Supplier;

/**
 * Make jobs.
 */
public class JobFactory {
    JobManager manager = null;
    Consumer<Job> onStart = null;
    Consumer<Job> task;
    Function<Job, Boolean> completeCondition;
    Consumer<Job> onComplete;
    int[] dependencies;

    public void reset() {
        manager = null;
        onStart = null;
        task = null;
        completeCondition = null;
        onComplete = null;
        dependencies = null;
    }

    /**
     * Set the job manager.
     *
     * @param manager The job manager to use
     */
    public JobFactory manager(JobManager manager) {
        this.manager = manager;
        return this;
    }

    /**
     * Runs when this Job is started.
     * Note that this isn't necessarily immediately after the Job is created, as
     * dependencies may need to be completed first.
     *
     * @param onStart The function to run
     */
    public JobFactory onStart(Consumer<Job> onStart) {
        this.onStart = onStart;
        return this;
    }

    /**
     * Runs when this Job is started.
     * Note that this isn't necessarily immediately after the Job is created, as
     * dependencies may need to be completed first.
     *
     * @param onStart The function to run
     */
    public JobFactory onStart(Runnable onStart) {
        this.onStart = job -> onStart.run();
        return this;
    }

    /**
     * Runs every iteration while this Job is active. DO NOT BLOCK.
     *
     * @param task The function to run
     */
    public JobFactory task(Consumer<Job> task) {
        this.task = task;
        return this;
    }

    /**
     * Runs every iteration while this Job is active. DO NOT BLOCK.
     *
     * @param task The function to run
     */
    public JobFactory task(Runnable task) {
        this.task = job -> task.run();
        return this;
    }

    /**
     * Runs every iteration, and returns true if the Job is complete.
     * DO NOT BLOCK.
     * When a Job is complete, it will become "finished" and will no longer run.
     *
     * @param completeCondition The function to run
     */
    public JobFactory completeCondition(Function<Job, Boolean> completeCondition) {
        this.completeCondition = completeCondition;
        return this;
    }

    /**
     * Runs every iteration, and returns true if the Job is complete.
     * DO NOT BLOCK.
     * When a Job is complete, it will become "finished" and will no longer run.
     *
     * @param completeCondition The function to run
     */
    public JobFactory completeCondition(Supplier<Boolean> completeCondition) {
        this.completeCondition = job -> completeCondition.get();
        return this;
    }

    /**
     * Runs when the Job is completed, either by the completeCondition returning true,
     * or the markComplete() method being called.
     *
     * @param onComplete The function to run
     */
    public JobFactory onComplete(Consumer<Job> onComplete) {
        this.onComplete = onComplete;
        return this;
    }

    /**
     * Runs when the Job is completed, either by the completeCondition returning true,
     * or the markComplete() method being called.
     *
     * @param onComplete The function to run
     */
    public JobFactory onComplete(Runnable onComplete) {
        this.onComplete = job -> onComplete.run();
        return this;
    }

    /**
     * Jobs that must be completed before this Job will start.
     * The jobs that are passed in will automatically start this one when all have completed.
     *
     * @param dependencies Job IDs that must be completed before this Job can start
     */
    public JobFactory dependencies(int[] dependencies) {
        this.dependencies = dependencies;
        return this;
    }

    public Job build() {
        if (manager == null) {
            throw new IllegalStateException("JobManager must be set");
        }

        Job j = new Job(manager, onStart, task, completeCondition, onComplete, dependencies);
        reset();
        return j;
    }
}
