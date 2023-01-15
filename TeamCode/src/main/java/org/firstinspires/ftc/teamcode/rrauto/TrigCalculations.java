package org.firstinspires.ftc.teamcode.rrauto;

// whiteboard drawing on engineering notebook 2023/01/07 (alpha and beta switched)

/*
 * coordinate frame with start at origin
 * +x is typical forward movement at start of auto
 * +y is moving left
 * -y is moving right
 *
 * (0, 0) start (center of turret)
 * (60, 0) expected stop position
 * (72, -12) pole position (B)
 * * (60, 36) cone stack position (C)

 * (x, y) measured stop position
 * -->  (assumes that this has been accurately measured from encoders)
 * */

public class TrigCalculations {

    private static final double startX = 9, startY = 0;
    private static final double stopX = 60, stopY = 0;
    private static final double poleX = 72, poleY = -12;    // pole
    private static final double stackX = 60, stackY = 36;     // cone stack

    // values adjusted for non-(0, 0) start position
    private static final double poleDiffX = poleX - startX, poleDiffY = poleY - startY;
    private static final double stackDiffX = stackX - startX, stackDiffY = stackY - startY;

    // account for distance from center of turret to front of claw
    private static final int HORIZONTAL_DISTANCE_DIFFERENCE_IN = 8;

    public static double initialDrive() {
        return stopX - startX;
    }

    public static double poleAngle(double x, double y) {
        double deltaX = poleDiffX - x;
        double deltaY = poleDiffY - y;

        // should return negative angle value
        return Math.toDegrees(Math.atan(deltaY / deltaX));
    }

    public static double stackAngle(double x, double y) {
        double deltaX = stackDiffX - x;
        double deltaY = stackDiffY - y;

        return Math.toDegrees(Math.atan(deltaY / deltaX));
    }

    public static double sumDeltaAngle(double x, double y) {
        // account for negative values from poleAngle()
        return Math.abs(poleAngle(x, y)) + Math.abs(stackAngle(x, y));
    }

    public static double distToPole(double x, double y) {
        double xComponent = poleDiffX - x;
        double yComponent = poleDiffY - y;

        double turretToPole = Math.sqrt(xComponent * xComponent + yComponent * yComponent);
        return turretToPole - HORIZONTAL_DISTANCE_DIFFERENCE_IN;
    }

    public static double distToStack(double x, double y) {
        double xComponent = stackDiffX - x;
        double yComponent = stackDiffY - y;

        double turretToStack = Math.sqrt(xComponent * xComponent + yComponent * yComponent);
        return turretToStack - HORIZONTAL_DISTANCE_DIFFERENCE_IN;
    }
}
