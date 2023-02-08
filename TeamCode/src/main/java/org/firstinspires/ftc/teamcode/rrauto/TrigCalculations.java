package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static java.lang.Math.abs;
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
    private static final double encToDist = 1900; //encoder counts per inch odometry wheels
    private static final double encToAngle = 750 / 90; //encoder counts per degree turret

    //ALL DISTANCES MEASURED FROM ORIGIN which is in center of turret
    private static final double startX = 10, startY = 0;
    private static final double stopX = encoderLeft.getCurrentPosition(), stopY = encoderRear.getCurrentPosition();
    private static final double poleX = 60.75, poleY = -12.4;    // pole
    private static final double stackX = 48.25, stackY = 35;     // cone stack

    // values adjusted for non-(0, 0) start position
    private static final double r1 = stackY - stopY, r2 = poleY - stopY;
    private static final double poleDiffX = poleX, poleDiffY = poleY;
    private static final double stackDiffX = stackX, stackDiffY = stackY;

    // account for distance from center of turret to front of claw
    private static final int HORIZONTAL_DISTANCE_DIFFERENCE_IN = 8;

    public static double initialDrive() {
        return stopX - startX;
    }

    public static double poleAngle(double x, double y) {
        double deltaX = poleDiffX - x/encToDist;
        double deltaY = poleDiffY + y/encToDist;

        // should return negative angle value
        return Math.toDegrees(Math.atan2(deltaX, Math.abs(deltaY)));
    }

    public static double stackAngle(double x, double y) {
        double deltaX = x / encToDist - stackDiffX;
        double deltaY = stackDiffY - y / encToDist;

        return (90 + Math.toDegrees(Math.atan2(deltaX, deltaY))) * encToAngle;
    }

    public static double stackAngleR(double x, double y) { //for right side
        double deltaX = x / encToDist - stackDiffX;
        double deltaY = (stackDiffY + 0.5) + y / encToDist;

        return (90 + Math.toDegrees(Math.atan2(deltaX, deltaY))) * encToAngle;
    }

    public static double sumDeltaAngle(double x, double y) {
        // account for negative values from poleAngle()
        return abs(poleAngle(x, y)) + abs(stackAngle(x, y));
    }

    public static double distToPole(double x, double y) {
        double xComponent = poleDiffX - x;
        double yComponent = poleDiffY - y;

        double turretToPole = Math.sqrt(xComponent * xComponent + yComponent * yComponent);
        return turretToPole - HORIZONTAL_DISTANCE_DIFFERENCE_IN;
    }

    public static double distToStack(double x, double y) {
        double xComponent = x / encToDist - stackDiffX;
        double yComponent = stackDiffY - y / encToDist;//9 inches is the distance from claw to center of turret

        double dist = Math.sqrt(xComponent * xComponent + yComponent * yComponent);
        return (dist-9) * 163 / 5.25;
    }

    public static double distToPoleMed(double x, double y) {
        double xComponent = x / encToDist - (poleDiffX - 24);
        double yComponent = abs(poleDiffY - y / encToDist);//9 inches is the distance from claw to center of turret
        double dist = Math.sqrt(xComponent * xComponent + yComponent * yComponent);
        return (dist - 9) * 163 / 5.25;
    }
    public static double distToPoleHigh(double x, double y) {
        double xComponent =(poleDiffX) - x / encToDist;
        double yComponent = abs(poleDiffY - y / encToDist);//9 inches is the distance from claw to center of turret
        double dist = Math.sqrt(xComponent * xComponent + yComponent * yComponent);
        return (dist - 9) * 163 / 5.25;
    }

}
