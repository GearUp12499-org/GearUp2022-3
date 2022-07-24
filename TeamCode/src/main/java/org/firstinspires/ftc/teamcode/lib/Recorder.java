package org.firstinspires.ftc.teamcode.lib;

import androidx.annotation.NonNull;

import java.util.ArrayList;
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
}
