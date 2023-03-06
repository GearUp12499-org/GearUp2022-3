package org.firstinspires.ftc.teamcode.rrauto;

import com.acmerobotics.dashboard.config.Config;

@Config
public class PID {
    private final double encoderConversion = 1700.0;
    private final double trackRadius = 6.5;
    public static double kp = 0.55;//0.1, 0.3
    public static double kd = 0.2;//1.5
    public static double ki = 0.0001;//0.01, 0.0175

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

        double error = target - theta;
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
        } else if (compensation < -maxCompensation) {
            compensation = -maxCompensation;
        }

        return compensation;
    }

    public double calculateTheta(double leftEnc, double rightEnc) {
        // s = r(theta) method
        // theta = backEnc / (trackRadius * encoderConversion);

        // small angle approximation method
        theta = (leftEnc - rightEnc) / (trackRadius * encoderConversion);
        return theta;
    }

    public double getTheta() {
        return theta;
    }
}
