package org.firstinspires.ftc.teamcode.lib.jobs;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.lib.Supplier;
import org.firstinspires.ftc.teamcode.lib.returns2;

public class ReturnJob<T> extends Job {
    /**
     * Create a new returing job, for running things "concurrently" with other jobs.<br/>
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
    public ReturnJob(@NonNull JobManager mgr, @Nullable Runnable onStart, @Nullable Runnable task, @Nullable Supplier<returns2<Boolean, T>> completeCondition, @Nullable Runnable onComplete, @Nullable int[] dependencies) {
        super(mgr, onStart, task, completeCondition, onComplete, dependencies);
    }
}
