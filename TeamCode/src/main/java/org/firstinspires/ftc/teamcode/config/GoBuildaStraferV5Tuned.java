package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class GoBuildaStraferV5Tuned extends GoBuildaStraferV5Base {
    public static double SLOWNESS = 2;

    @Override
    public double getMaxVel() {
        return super.getMaxVel()/SLOWNESS;
    }

    @Override
    public double getMaxAccel() {
        return super.getMaxAccel()/SLOWNESS;
    }

    @Override
    public double getTrackWidth() {
        return 18.5;
    }

    @Override
    public double getXMultiplier() {
        // 89.25
        // 89.19
        // 89
        return 1.009572241;
    }

    @Override
    public double getYMultiplier() {
        // 88.95
        // 89.63
        // 88.72
        return 1.011292769;
    }

    @Override
    public double getKV() {
        return 0.0191960882997653;
    }

    @Override
    public double getKA() {
        return 0.0022;
    }

    @Override
    public double getMaxAngVel() {
        return 8.021820929253712;
    }

    @Override
    public double getLateralMultiplier() {
        return 4; // FIXME! today
    }

    @Override
    public PIDCoefficients getHeadingPID() {
        return new PIDCoefficients(2.5, 0, 0);
    }

    @Override
    public PIDCoefficients getTranslationalPID() {
        return new PIDCoefficients(2, 0, 0);
    }

    @Override
    public double getEncoderLateralDistance() {
        return 16.24;
    }
}
