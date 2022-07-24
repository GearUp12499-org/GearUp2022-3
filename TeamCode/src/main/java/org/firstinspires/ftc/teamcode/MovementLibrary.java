package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MovementLibrary {
    public static class MotorEtcConf {
        DcMotor frontLeft;
        DcMotor frontRight;
        DcMotor rearLeft;
        DcMotor rearRight;

        DcMotor odForward1;
        DcMotor odForward2;
        DcMotor odSide;
        DcMotor odRear;

    }

    public MovementLibrary(MotorEtcConf configuration) {}

    // To compensate for differences in field setup
    // definitely not a cheap hack that wont work in the long run
    private static final int TUNER = -3000;

    /**
     * @param distance Meters
     * @param power Motor power to use
     * @return
     */
    public static double getTargetEncoder(double distance, double power) {
        double x = (Math.pow(power, 2) * 1.340964) / 2.18;
        double distBeforeBraking = distance - x;
        double targetEncoderCounts = distBeforeBraking/0.0000134;
        return targetEncoderCounts + TUNER;
    }
}
