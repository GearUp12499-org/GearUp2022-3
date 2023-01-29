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

    public JobFactory manager(JobManager manager) {
        this.manager = manager;
        return this;
    }

    public JobFactory onStart(Consumer<Job> onStart) {
        this.onStart = onStart;
        return this;
    }

    public JobFactory onStart(Runnable onStart) {
        this.onStart = job -> onStart.run();
        return this;
    }

    public JobFactory task(Consumer<Job> task) {
        this.task = task;
        return this;
    }

    public JobFactory task(Runnable task) {
        this.task = job -> task.run();
        return this;
    }

    public JobFactory completeCondition(Function<Job, Boolean> completeCondition) {
        this.completeCondition = completeCondition;
        return this;
    }

    public JobFactory completeCondition(Supplier<Boolean> completeCondition) {
        this.completeCondition = job -> completeCondition.get();
        return this;
    }

    public JobFactory onComplete(Consumer<Job> onComplete) {
        this.onComplete = onComplete;
        return this;
    }

    public JobFactory onComplete(Runnable onComplete) {
        this.onComplete = job -> onComplete.run();
        return this;
    }

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
