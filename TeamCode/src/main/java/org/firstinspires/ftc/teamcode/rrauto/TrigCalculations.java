package org.firstinspires.ftc.teamcode.rrauto;

// whiteboard drawing on engineering notebook 2023/01/07 (alpha and beta switched)

/*
 * coordinate frame with start at origin
 * +x is typical forward movement at start of auto
 * +y is moving left
 * -y is moving right
 *
 * (0, 0) start
 * (60, 0) expected stop position
 * (72, -12) pole position (B)
 * * (60, 36) cone stack position (C)

 * (x, y) measured stop position
 * -->  (assumes that this has been accurately measured from encoders)
 * */

public class TrigCalculations {

    private static final double poleX = 72, poleY = -12;    // pole
    private static final double stackX = 60, stackY = 36;     // cone stack

    private static double poleAngle(int x, int y) {
        double deltaX = poleX - x;
        double deltaY = Math.abs(poleY - y);
        return Math.atan(deltaY / deltaX);
    }

    private static double stackAngle(int x, int y) {
        double deltaX = stackX - x;
        double deltaY = stackY - y;
        return Math.atan(deltaY / deltaX);
    }

    public static double angleDelta(int x, int y){
        return poleAngle(x, y) + stackAngle(x, y);
    }

    public static double distToPole(int x, int y) {
        double xComponent = poleX - x;
        double yComponent = Math.abs(poleY - y);
        return Math.sqrt(xComponent * xComponent + yComponent * yComponent);
    }

    public static double distToStack(int x, int y) {
        double xComponent = stackX - x;
        double yComponent = stackY - y;
        return Math.sqrt(xComponent * xComponent + yComponent * yComponent);
    }
}
