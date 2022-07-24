package org.firstinspires.ftc.teamcode.lib;

import android.content.Context;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Hashmap wrapper because... yeah
 */
public class Recorder {
    public enum State {
        OPEN,
        CLOSED
    }

    public Map<Double, Double> storage = new HashMap<>();

    public double FINISH_EXTEND_TIME = 2.0;

    private boolean exported = false;

    private State state = State.OPEN;
    public State getState() {
        return state;
    }

    private boolean disallowFilter = false;
    public double disallowAfter = 0.0;

    public Recorder() {}

    public void put(double time, double data) {
        if (disallowFilter && time > disallowAfter) return;
        storage.put(time, data);
    }

    public void finalize(double secondsNow) {
        disallowAfter = secondsNow + FINISH_EXTEND_TIME;
        disallowFilter = true;
        state = State.CLOSED;
    }

    @NonNull @Override
    public String toString() {
        StringBuilder b = new StringBuilder();
        b.append('{');
        String p = "";
        for (Map.Entry<Double, Double> entry : storage.entrySet()) {
            b.append(p);
            p = ", ";
            b.append(entry.getKey().toString()).append(": ").append(entry.getValue().toString());
        }
        b.append('}');
        return b.toString();
    }

    @NonNull
    public String toCSV() {
        StringBuilder b = new StringBuilder();
        b.append("Time,Value\n");
        for (Map.Entry<Double, Double> entry : storage.entrySet()) {
            b.append(entry.getKey().toString()).append(",").append(entry.getValue().toString()).append('\n');
        }
        return b.toString();
    }

    public List<Map<Double, Double>> sliceIntoPartsOf(int count) {
        List<Map<Double, Double>> parts = new ArrayList<>();
        // retrieve data, {count} at a time
        int i = 0;
        Map<Double, Double> builder = new HashMap<>();
        for (Map.Entry<Double, Double> entry : storage.entrySet()) {
            if (++i % count == 0) {
                parts.add(builder);
                builder.clear();
            }
            builder.put(entry.getKey(), entry.getValue());
        }
        return parts;
    }

    public String combinedStateString() {
        StringBuilder b = new StringBuilder();
        switch (state) {
            case OPEN:
                b.append("Open");
                break;
            case CLOSED:
                b.append("Closed");
                break;
        }
        b.append(exported ? ", Exported" : "");
        b.append(storage.size() > 0 ? (", " + storage.size() + " items") : ", <empty>");
        return b.toString();
    }

    public List<String> writeIfClosed(double secondsNow, String target) {
        if (disallowFilter && secondsNow > disallowAfter) return writeOnce(target);
        return Collections.singletonList("Not closed");
    }

    public List<String> writeOnce(String target) {
        if (exported) return Collections.singletonList("Already exported. Set exported to false to re-export.");
        return write(target);
    }

    public List<String> write(String target) {
        List<String> log = new ArrayList<>();
        Context context = AppUtil.getInstance().getApplication();
        String data = toCSV();
        File file = new File(context.getFilesDir(), target);
        if (file.delete()) {
            log.add("Deleted old file");
        }
        try (FileOutputStream fos = context.openFileOutput(target, Context.MODE_PRIVATE)) {
            fos.write(data.getBytes(StandardCharsets.UTF_8));
            log.add("Write " + data.length() + " bytes to " + target);
        } catch (IOException e) {
            log.add("Failed to write file");
        }
        exported = true;
        return log;
    }
}
