package org.firstinspires.ftc.teamcode.rrauto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class PID {
    private final double encoderConversion = 1700.0;
    private final double trackRadius = 6.5;
    private final double kp = 7;
    private final double kd = 7;
    private final double ki = 7;

    private final double maxThetaChange = 10;

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
        return theta;
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
