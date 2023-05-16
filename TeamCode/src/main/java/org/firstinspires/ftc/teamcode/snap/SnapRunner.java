package org.firstinspires.ftc.teamcode.snap;

import java.util.ArrayList;

public class SnapRunner {
    public ArrayList<Snap> snaps = new ArrayList<Snap>();

    public SnapRunner() {
    }

    public void addSnap(Snap snap) {
        snaps.add(snap);
    }

    public void init() {
        for (Snap snap : snaps) {
            snap.init();
        }
    }

    public void loop() {
        for (Snap snap : snaps) {
            snap.loop();
        }
    }

    public void finish() {
        for (Snap snap : snaps) {
            snap.finish();
        }
    }
}
