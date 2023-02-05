package org.firstinspires.ftc.teamcode.lib.jobs;

import android.content.Context;

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
        data = "Job Timing Profile\n\n";
        for (Job job : sources) {
            StringBuilder b = new StringBuilder();
            b.append("ID ").append(job.id).append(" (").append(job.toString()).append(")").append("\n");
            Job.Timings timings = job.timings;
            b.append(" ").append("Created=").append(timings.getCreatedAt()).append("\n");
            b.append(" ").append("Started=").append(timings.getStartedAt()).append("\n");
            b.append(" ").append("Completed=").append(timings.getCompletedAt()).append("\n");
            b.append(" ").append("WaitingDuration=").append(timings.waitingToStartTime())
                    .append(" (").append(Job.Timings.formatDuration(timings.waitingToStartTime())).append(")\n");
            b.append(" ").append("RunningDuration=").append(timings.runningOverallTime())
                    .append(" (").append(Job.Timings.formatDuration(timings.runningOverallTime())).append(")\n");
            b.append(" ").append("TotalExecutingTime=").append(timings.getTotalTimeRunning())
                    .append(" (").append(Job.Timings.formatDuration(timings.getTotalTimeRunning())).append(")\n");
            b.append(" ").append("MeanTaskTime=").append(timings.averageTaskMs())
                    .append(" (").append(Job.Timings.formatDuration(timings.averageTaskMs())).append(")\n");
            b.append(" ").append("P95TaskTime=").append(timings.p95TaskMs())
                    .append(" (").append(Job.Timings.formatDuration(timings.p95TaskMs())).append(")\n");
            b.append(" ").append("MaxTaskTime=").append(timings.maxTaskMs())
                    .append(" (").append(Job.Timings.formatDuration(timings.maxTaskMs())).append(")\n");
            b.append(" ").append("Blocking=").append(Arrays.toString(timings.getDownstreamAtFinish())).append("\n");
            b.append(" ").append("AllTaskTimes=").append(timings.getTaskTimes().toString()).append("\n");
            b.append("\n");
            data += b.toString();
        }
    }

    public boolean isBuilt() {
        return built;
    }

    public void export(String target, boolean deleteOK) {
        if (!built)
            throw new IllegalStateException("ProfileFile not built");

        Context context = AppUtil.getInstance().getApplication();
        File file = new File(context.getFilesDir(), target);
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
