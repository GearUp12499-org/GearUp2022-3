package org.firstinspires.ftc.teamcode.lib.jobs;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.lib.Consumer;
import org.firstinspires.ftc.teamcode.lib.Function;
import org.firstinspires.ftc.teamcode.lib.NullTools;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Represents a job that can be run in a JobManager.
 */
public class Job {
    public static final boolean TIMINGS = true;
    // ms, if the <insert variable> is greater than this, the job will be considered "slow"
    // and output warnings when complete
    public static final double MAX_MEAN_TASK_TIME = 100; // 10 i/s
    public static final double MAX_95TH_TASK_TIME = 150; // 6 + 2/3 i/s
    public static final double MAX_MAX_TASK_TIME = 250; // 4 i/s

    public static class Timings {
        @SuppressLint("DefaultLocale")
        public static String formatDuration(long ms) {
            if (ms < 1000) return ms + "ms";
            double seconds = ms / 1000.0;
            if (seconds < 60) return String.format("%.2fs", seconds);
            double minutes = seconds / 60.0;
            if (minutes < 60) return String.format("%dm %.2fs", (int) Math.floor(minutes), seconds % 60);
            double hours = minutes / 60.0;
            return String.format("%dh %dm %.2fs", (int) Math.floor(hours), (int) Math.floor(minutes % 60), seconds % 60);
        }

        @SuppressLint("DefaultLocale")
        public static String formatDuration(double ms) {
            if (ms < 1000) return String.format("%.1fms", ms);
            double seconds = ms / 1000.0;
            if (seconds < 60) return String.format("%.2fs", seconds);
            double minutes = seconds / 60.0;
            if (minutes < 60) return String.format("%dm %.2fs", (int) Math.floor(minutes), seconds % 60);
            double hours = minutes / 60.0;
            return String.format("%dh %dm %.2fs", (int) Math.floor(hours), (int) Math.floor(minutes % 60), seconds % 60);
        }

        public long getCreatedAt() {
            return createdAt;
        }

        public long getStartedAt() {
            return startedAt;
        }

        public long getCompletedAt() {
            return completedAt;
        }

        public long getTotalTimeRunning() {
            return totalTimeRunning;
        }

        public long getTotalTimeWaiting() {
            return totalTimeWaiting;
        }

        public boolean isSealed() {
            return sealed;
        }

        /**
         * @return Duration from the creation of the Job to when it is started.
         */
        public long waitingToStartTime() {
            return startedAt - createdAt;
        }

        /**
         * @return Duration during which the job has the "running" state.
         */
        public long runningOverallTime() {
            return completedAt - startedAt;
        }

        /**
         * @return List of times, in milliseconds, that the job took to complete its task and comparator.
         */
        public ArrayList<Long> getTaskTimes() {
            return taskTimes;
        }

        public double averageTaskMs() {
            long sum = 0;
            for (long taskTime : taskTimes) {
                sum += taskTime;
            }
            if (taskTimes.size() == 0) return 0;
            return sum / (double) taskTimes.size();
        }

        private ArrayList<Long> sortedTaskTimes() {
            ArrayList<Long> sorted = new ArrayList<>(taskTimes);
            Collections.sort(sorted, Long::compare);
            return sorted;
        }

        public double medianTaskMs() {
            ArrayList<Long> sorted = sortedTaskTimes();
            if (sorted.size() == 0) return 0;
            return sorted.get(sorted.size() / 2);
        }

        /**
         * 95th percentile task time, in milliseconds.
         * @return 95th percentile task time, in milliseconds.
         */
        public double p95TaskMs() {
            ArrayList<Long> sorted = sortedTaskTimes();
            if (sorted.size() == 0) return 0;
            return sorted.get((int) (sorted.size() * 0.95));
        }

        /**
         * Absolute maximum task time, in milliseconds.
         */
        public long maxTaskMs() {
            ArrayList<Long> sorted = sortedTaskTimes();
            if (sorted.size() == 0) return 0;
            return sorted.get(sorted.size() - 1);
        }

        private final long createdAt;
        private long startedAt = -1;
        private long completedAt = -1;

        private long totalTimeRunning = -1;
        private long totalTimeWaiting = -1;

        private long lastIdleInvoke = -1;
        private long taskTimeStart = -1;

        private boolean sealed = false;
        private final ArrayList<Long> taskTimes = new ArrayList<>();
        private void notWhenSealed() {
            if (sealed) throw new IllegalStateException("Timings sealed");
        }

        public Timings() {
            createdAt = System.currentTimeMillis();
        }

        public void start() {
            notWhenSealed();
            startedAt = System.currentTimeMillis();
        }

        public void complete() {
            notWhenSealed();
            completedAt = System.currentTimeMillis();
        }

        public void bump_waiting() {
            notWhenSealed();
            long now = System.currentTimeMillis();
            if (lastIdleInvoke != -1) {
                totalTimeWaiting += now - lastIdleInvoke;
            }
            lastIdleInvoke = now;
        }

        public void bump_start_task() {
            notWhenSealed();
            taskTimeStart = System.currentTimeMillis();
        }

        public void bump_end_task() {
            notWhenSealed();
            long now = System.currentTimeMillis();
            if (taskTimeStart != -1) {
                taskTimes.add(now - taskTimeStart);
                totalTimeRunning += now - taskTimeStart;
            }
        }
    }

    public final int id;
    private final JobManager manager;
    public Timings timings = new Timings();
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
        if (TIMINGS) {
            timings.start();
        }
        this.active = true;
    }

    /**
     * Complete appropriate actions based on the state.
     */
    public void tick() {
        if (isActive()) {
            if (TIMINGS) {
                timings.bump_start_task();
            }
            this.task.accept(this);
            if (completeCondition.apply(this)) {
                completeHandler();
            }
            if (TIMINGS) {
                timings.bump_end_task();
            }
        } else if (TIMINGS) {
            timings.bump_waiting();
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
        if (TIMINGS) {
            timings.complete();
            long now = System.currentTimeMillis();
            RobotLog.ii("JobTimings", manager.labelFor(id) + " timings:");
            RobotLog.ii("JobTimings", "  Lifecycle time: " + Timings.formatDuration(timings.waitingToStartTime() + timings.runningOverallTime()));
            RobotLog.ii("JobTimings", "   ┣ Waiting to start: " + Timings.formatDuration(timings.waitingToStartTime()));
            RobotLog.ii("JobTimings", "   ┗ Running overall : " + Timings.formatDuration(timings.runningOverallTime()));
            RobotLog.ii("JobTimings", "      ┗ This task    : " + Timings.formatDuration(timings.getTotalTimeRunning()));
            RobotLog.ii("JobTimings", "  Created " + Timings.formatDuration(now - timings.getCreatedAt()) + " ago");
            RobotLog.ii("JobTimings", "  Started " + Timings.formatDuration(now - timings.getStartedAt()) + " ago");
            RobotLog.ii("JobTimings", "  Mean task time: " + Timings.formatDuration(timings.averageTaskMs()));
            RobotLog.ii("JobTimings", "   ┃ (" + timings.getTaskTimes().size() + " samples)");
            RobotLog.ii("JobTimings", "   ┣ 95% median : " + Timings.formatDuration(timings.p95TaskMs()));
            RobotLog.ii("JobTimings", "   ┗ Maximum    : " + Timings.formatDuration(timings.maxTaskMs()));
            ArrayList<String> problems = new ArrayList<>();
            if (timings.averageTaskMs() > MAX_MEAN_TASK_TIME) {
                problems.add("Job is too slow on average (mean task time is " + Timings.formatDuration(timings.averageTaskMs()) + ", configured limit is " + Timings.formatDuration(MAX_MEAN_TASK_TIME) + ")");
            }
            if (timings.p95TaskMs() > MAX_95TH_TASK_TIME) {
                problems.add("Job is too slow at peaks (95% median task time is " + Timings.formatDuration(timings.p95TaskMs()) + ", configured limit is " + Timings.formatDuration(MAX_95TH_TASK_TIME) + ")");
            }
            if (timings.maxTaskMs() > MAX_MAX_TASK_TIME) {
                problems.add("Task ran for " + Timings.formatDuration(timings.maxTaskMs()) + " (max), configured limit is " + Timings.formatDuration(MAX_MAX_TASK_TIME));
            }
            if (!problems.isEmpty()) {
                RobotLog.ee("JobTimings", "Performance issues detected:");
                for (String problem : problems) {
                    RobotLog.ee("JobTimings", "  " + problem);
                }
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
     * Invoke a Function in a way that looks nice
     * @param supplier function adding additional jobs in a chain
     * @return next job
     */
    public Job jobSequence(Function<Job, Job> supplier) {
        return supplier.apply(this);
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
     * Run another Job after this Job, but don't return that one and keep working on this one.
     * @param b Job to run after this one.
     * @return This job
     */
    public Job andThenAsync(Job b) {
        andThen(b);
        return this;
    }

    /**
     * Find all jobs that we depend on and have no other dependencies, and start them.
     */
    public void start() {
        List<Integer> frontier = new ArrayList<>();
        List<Integer> done = new ArrayList<>();
        frontier.add(id);
        int i = 0;
        while (frontier.size() != 0) {
            i++;
            Job current = manager.getJob(frontier.get(0));
            frontier.remove(0);
            done.add(current.id);
            if (current.isComplete() || current.isActive()) {
                continue;
            }
            ArrayList<Integer> dependencies = current.getDependencies();
            if (dependencies.size() == 0) {
                current.startHandler();
                continue;
            }
            for (int dependency : dependencies) {
                if (!done.contains(dependency)) {
                    frontier.add(dependency);
                }
            }
            if (i > 1000) throw new RuntimeException("Infinite loop detected in start(), aborting");
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

    @NonNull
    @Override
    public String toString() {
        if (manager == null) {
            return "[" + (isActive() ? "running" : isComplete() ? "complete" : "waiting") + " " + id + "]";
        }
        return "[" + (isActive() ? "running" : isComplete() ? "complete" : "waiting") + " " + manager.labelFor(id) + "]";
    }
}
