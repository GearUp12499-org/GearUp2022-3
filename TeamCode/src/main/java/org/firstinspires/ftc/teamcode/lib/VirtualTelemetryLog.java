package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class VirtualTelemetryLog {
    private final int size;
    private final List<String> buffer = new ArrayList<>();

    public VirtualTelemetryLog(int size) {
        this.size = size;
    }

    public void put(String log) {
        buffer.add(log);
        if (buffer.size() > size) {
            buffer.remove(0);
        }
    }

    public void put(String format, Object... args) {
        put(String.format(format, args));
    }

    public void dump(Telemetry telemetry) {
        for (String log : buffer) {
            telemetry.addLine(log);
        }
    }
}
