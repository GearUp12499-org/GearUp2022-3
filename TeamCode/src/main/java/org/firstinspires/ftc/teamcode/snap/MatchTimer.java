package org.firstinspires.ftc.teamcode.snap;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MatchTimer implements Snap {
    private final ElapsedTime timer = new ElapsedTime();
    public static final double MATCH_TIME = 120;
    private final Telemetry telemetry;

    public MatchTimer(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public String fmtDuration(double duration) {
        int minutes = (int) (duration / 60);
        int seconds = (int) (duration % 60);
        int milliseconds = (int) ((duration - (int) duration) * 1000);
        return String.format("%02d:%02d.%03d", minutes, seconds, milliseconds);
    }

    @Override
    public void init() {
        timer.reset();
    }

    @Override
    public void loop() {
        this.telemetry.addData("Match timer", fmtDuration(MATCH_TIME - timer.seconds()));
    }

    @Override
    public void finish() {

    }
}
