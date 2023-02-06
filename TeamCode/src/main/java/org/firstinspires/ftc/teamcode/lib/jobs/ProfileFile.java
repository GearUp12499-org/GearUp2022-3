package org.firstinspires.ftc.teamcode.lib.jobs;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;

public class ProfileFile {
    private final List<Job> sources;
    private String data = "";
    private boolean built = false;

    public ProfileFile(Collection<Job> sources) {
        this.sources = new ArrayList<>(sources);
    }

    public void build() {
        if (built) throw new IllegalStateException("ProfileFile already built");
        StringBuilder c = new StringBuilder("Job Timing Profile\n\n");
        for (Job job : sources) {
            Job.Timings timings = job.timings;
            String b = "ID " + job.id + " (" + job + ")" + "\n" +
                    " " + "Created=" + timings.getCreatedAt() + "\n" +
                    " " + "Started=" + timings.getStartedAt() + "\n" +
                    " " + "Completed=" + timings.getCompletedAt() + "\n" +
                    " " + "WaitingDuration=D" + timings.waitingToStartTime() +
                    " (" + Job.Timings.formatDuration(timings.waitingToStartTime()) + ")\n" +
                    " " + "RunningDuration=D" + timings.runningOverallTime() +
                    " (" + Job.Timings.formatDuration(timings.runningOverallTime()) + ")\n" +
                    " " + "TotalExecutingTime=D" + timings.getTotalTimeRunning() +
                    " (" + Job.Timings.formatDuration(timings.getTotalTimeRunning()) + ")\n" +
                    " " + "TaskTimeSamples=" + timings.getTaskTimes().size() + "\n" +
                    " " + "MeanTaskTime=D" + timings.averageTaskMs() +
                    " (" + Job.Timings.formatDuration(timings.averageTaskMs()) + ")\n" +
                    " " + "P95TaskTime=D" + timings.p95TaskMs() +
                    " (" + Job.Timings.formatDuration(timings.p95TaskMs()) + ")\n" +
                    " " + "MaxTaskTime=D" + timings.maxTaskMs() +
                    " (" + Job.Timings.formatDuration(timings.maxTaskMs()) + ")\n" +
                    " " + "Blocking=A" + Arrays.toString(timings.getDownstreamAtFinish()) + "\n" +
                    " " + "AllTaskTimes=A" + timings.getTaskTimes().toString() + "\n" +
                    "\n";
            c.append(b);
        }
        data = c.toString();
        built = true;
    }

    public boolean isBuilt() {
        return built;
    }

    @SuppressWarnings("ResultOfMethodCallIgnored")
    public void export(String target, boolean deleteOK) {
        if (!built)
            throw new IllegalStateException("ProfileFile not built");

        Context context = AppUtil.getInstance().getApplication();
        File file = new File(context.getFilesDir(), target);
        RobotLog.ii("ProfileFile", "saving to " + file.getAbsolutePath());
        if (file.exists()) {
            if (deleteOK && file.isFile()) file.delete();
            else if (file.isFile())
                throw new IllegalArgumentException("Provided File already exists and deleteOK is false");
            else
                throw new IllegalArgumentException("Provided File is not a file");
        }

        try {
            file.setWritable(true);
            file.setReadable(true);

            try (FileOutputStream fos = context.openFileOutput(target, Context.MODE_PRIVATE)) {
                fos.write(data.getBytes(StandardCharsets.UTF_8));
            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

    public void export(String s) {
        export(s, false);
    }
}
