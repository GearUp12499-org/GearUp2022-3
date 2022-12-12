package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class NewTuningTuned extends NewTuningBase {

    @Override
    public double getTrackWidth() {
        return super.getTrackWidth(); // TODO
    }

    @Override
    public double getXMultiplier() {
        return 1; // TODO
    }

    @Override
    public double getYMultiplier() {
        return 1; // TODO
    }

    @Override
    public double getKV() {
        return 0;
    }

    @Override
    public double getKA() {
        return 0;
    }

    @Override
    public double getMaxAngVel() {
        return super.getMaxAngVel();
    }

    @Override
    public double getLateralMultiplier() {
        return 1; // TODO
    }

    @Override
    public PIDCoefficients getHeadingPID() {
        return new PIDCoefficients(0, 0, 0);
    }

    @Override
    public PIDCoefficients getTranslationalPID() {
        return new PIDCoefficients(0, 0, 0);
    }

    @Override
    public double getEncoderLateralDistance() {
        return super.getEncoderLateralDistance();
    }
}
