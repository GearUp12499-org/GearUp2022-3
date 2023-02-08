package org.firstinspires.ftc.teamcode.lib.jobs;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.lib.Consumer;
import org.firstinspires.ftc.teamcode.lib.Function;

/**
 * Job that returns a value.
 * @param <T> The type of the value.
 */
public class ResultJob<T> extends Job{
    private T value;

    public T getResult() {
        return value;
    }

    public void setResult(T value) {
        this.value = value;
    }

    public ResultJob(
            @NonNull JobManager mgr,
            @Nullable Consumer<Job> onStart,
            @Nullable Consumer<Job> task,
            @Nullable Function<Job, Boolean> completeCondition,
            @Nullable Consumer<Job> onComplete,
            @Nullable int[] dependencies
    ) {
        super(mgr, onStart, task, completeCondition, onComplete, dependencies);
    }
}
