package org.firstinspires.ftc.teamcode.rrauto;

public class PID {
    private static final double trackRadius = 6.5;
    private static final double kp = 7;
    private static final double kd = 7;
    private static final double ki = 7;

    private static final double maxThetaChange = 10;

    private double errorPrev = 0;
    private double deltaError = 0;
    private double errorSum = 0;

    private double target;
    private double theta;

    public PID(double target, double theta) {
        this.target = target;
        this.theta = theta;
    }

    public static double pidUpdate() {
        double error = target - theta;
        double deltaTheta = 0.0;

        // proportional
        deltaTheta += kp * error;

        // derivative
        deltaError = errorPrev - error;
        deltaTheta += kd * deltaError;
        errorPrev = error;

        // integral
        errorSum += error;
        deltaTheta += ki * errorSum;

        // enforce max change per iteration
        if(deltaTheta > maxThetaChange) {
            deltaTheta = maxThetaChange;
        } else if (deltaTheta < -1 * maxThetaChange) {
            deltaTheta = -1 * maxThetaChange;
        }

        theta += deltaTheta;        
    }

    public static void updateTheta(double leftEnc, double rightEnc) {
        theta = Math.asin((rightEnc - leftEnc) / (2 * trackRadius));
    }

    public static double getTheta() {
        return theta;
    }
}
