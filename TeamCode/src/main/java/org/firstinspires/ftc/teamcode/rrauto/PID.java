package org.firstinspires.ftc.teamcode.rrauto;

public class PID {
    private final double encoderConversion = 1700.0;
    private final double trackRadius = 6.5;
    private final double kp = 0.10;//0.1, 0.3
    private final double kd =1.5;//1.5
    private final double ki = 0.0175;//0.01

    private final double maxCompensation = 99;

    private double errorPrev = 0;
    private double deltaError = 0;
    private double errorSum = 0;

    private double target;
    private double theta;

    public PID(double target) {
        this.target = target;
    }

    public double pidUpdate() {

        double error = -1 * (target - theta);
        double compensation = 0.0;

        // proportional
        compensation += kp * error;

        // derivative
        deltaError = errorPrev - error;
        compensation += kd * deltaError;
        errorPrev = error;

        // integral
        errorSum += error;
        compensation += ki * errorSum;

        // enforce max change per iteration
        if(compensation > maxCompensation) {
            compensation = maxCompensation;
        } else if (compensation < -1 * maxCompensation) {
            compensation = -1 * maxCompensation;
        }

        theta -= compensation;

        return -compensation;
    }

    public double calculateTheta(double backEnc) {
        // s = r(theta)
        theta = backEnc / (trackRadius * encoderConversion);
        return theta;
    }

    public double getTheta() {
        return theta;
    }
}
