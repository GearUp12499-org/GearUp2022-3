package org.firstinspires.ftc.teamcode.lib.jobs;

import org.firstinspires.ftc.teamcode.lib.Consumer;
import org.firstinspires.ftc.teamcode.lib.Function;
import org.firstinspires.ftc.teamcode.lib.Supplier;

/**
 * Make result jobs.
 */
public class ResultJobFactory<T> extends JobFactory {
    @Override
    public ResultJobFactory<T> manager(JobManager manager) {
        super.manager(manager);
        return this;
    }

    @Override
    public ResultJobFactory<T> onStart(Consumer<Job> onStart) {
        super.onStart(onStart);
        return this;
    }

    @Override
    public ResultJobFactory<T> onStart(Runnable onStart) {
        super.onStart(onStart);
        return this;
    }

    @Override
    public ResultJobFactory<T> task(Consumer<Job> task) {
        super.task(task);
        return this;
    }

    @Override
    public ResultJobFactory<T> task(Runnable task) {
        super.task(task);
        return this;
    }

    @Override
    public ResultJobFactory<T> completeCondition(Function<Job, Boolean> completeCondition) {
        super.completeCondition(completeCondition);
        return this;
    }

    @Override
    public ResultJobFactory<T> completeCondition(Supplier<Boolean> completeCondition) {
        super.completeCondition(completeCondition);
        return this;
    }

    @Override
    public ResultJobFactory<T> onComplete(Consumer<Job> onComplete) {
        super.onComplete(onComplete);
        return this;
    }

    @Override
    public ResultJobFactory<T> onComplete(Runnable onComplete) {
        super.onComplete(onComplete);
        return this;
    }

    @Override
    public ResultJobFactory<T> dependencies(int[] dependencies) {
        super.dependencies(dependencies);
        return this;
    }

    @Override
    public ResultJob<T> build() {
        if (manager == null) {
            throw new IllegalStateException("JobManager must be set");
        }

        ResultJob<T> j = new ResultJob<>(manager, onStart, task, completeCondition, onComplete, dependencies);
        reset();
        return j;
    }
}
