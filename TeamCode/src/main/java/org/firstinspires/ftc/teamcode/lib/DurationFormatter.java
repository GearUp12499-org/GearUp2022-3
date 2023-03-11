package org.firstinspires.ftc.teamcode.lib;

import android.annotation.SuppressLint;

public class DurationFormatter {
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
}
