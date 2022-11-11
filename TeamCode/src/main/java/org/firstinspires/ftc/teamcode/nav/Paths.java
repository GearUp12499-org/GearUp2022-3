package org.firstinspires.ftc.teamcode.nav;

public class Paths {
    private final EncoderNavigation nav;
    public static final double TILE = 24;

    public Paths(EncoderNavigation nav) {
        this.nav = nav;
    }

    /**
     * move from Start to Signal Zone 1
     */
    public void zone1() {
        nav.strafeLeft(TILE);
        nav.moveForward(1.5 * TILE);
    }

    /**
     * move from Start to Signal Zone 2
     */
    public void zone2() {
        nav.moveForward(1.5 * TILE);
    }

    /**
     * move from Start to Signal Zone 3
     */
    public void zone3() {
        nav.strafeRight(TILE);
        nav.moveForward(1.5 * TILE);
    }
}
