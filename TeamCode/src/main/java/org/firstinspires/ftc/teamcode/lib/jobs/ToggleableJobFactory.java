package org.firstinspires.ftc.teamcode.lib.jobs;

import org.firstinspires.ftc.teamcode.lib.Consumer;
import org.firstinspires.ftc.teamcode.lib.Function;
import org.firstinspires.ftc.teamcode.lib.Supplier;

/**
 * Make result jobs.
 */
public class ToggleableJobFactory extends JobFactory {
    @Override
    public ToggleableJobFactory manager(JobManager manager) {
        super.manager(manager);
        return this;
    }

    @Override
    public ToggleableJobFactory onStart(Consumer<Job> onStart) {
        super.onStart(onStart);
        return this;
    }

    @Override
    public ToggleableJobFactory onStart(Runnable onStart) {
        super.onStart(onStart);
        return this;
    }

    @Override
    public ToggleableJobFactory task(Consumer<Job> task) {
        super.task(task);
        return this;
    }

    @Override
    public ToggleableJobFactory task(Runnable task) {
        super.task(task);
        return this;
    }

    @Override
    public ToggleableJobFactory completeCondition(Function<Job, Boolean> completeCondition) {
        throw new UnsupportedOperationException("Toggleable jobs do not support complete conditions");
    }

    @Override
    public ToggleableJobFactory completeCondition(Supplier<Boolean> completeCondition) {
        throw new UnsupportedOperationException("Toggleable jobs do not support complete conditions");
    }

    @Override
    public ToggleableJobFactory onComplete(Consumer<Job> onComplete) {
        super.onComplete(onComplete);
        return this;
    }

    @Override
    public ToggleableJobFactory onComplete(Runnable onComplete) {
        super.onComplete(onComplete);
        return this;
    }

    @Override
    public ToggleableJobFactory dependencies(int[] dependencies) {
        super.dependencies(dependencies);
        return this;
    }

    @Override
    public ToggleableJob build() {
        if (manager == null) {
            throw new IllegalStateException("JobManager must be set");
        }

        ToggleableJob j = new ToggleableJob(manager, onStart, task, onComplete);
        reset();
        return j;
    }
}
