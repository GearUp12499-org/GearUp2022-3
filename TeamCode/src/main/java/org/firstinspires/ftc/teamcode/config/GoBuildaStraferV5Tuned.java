package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.roadrunner.control.PIDCoefficients;

public class GoBuildaStraferV5Tuned extends GoBuildaStraferV5Base {
    public static double SLOWDOWN = 1;

    @Override
    public double getMaxVel() {
        return super.getMaxVel() * SLOWDOWN;
    }

    @Override
    public double getMaxAccel() {
        return super.getMaxAccel();
    }


    @Override
    public double getTrackWidth() {
        return 13.5;
    } // real: 14.9

    @Override
    public double getXMultiplier() {
        return 1.0062;
    }//1.0074

    @Override
    public double getYMultiplier() {
        return 1.0064;
    }

    @Override
    public double getKV() {
        return 0.02;
    } //0.01841

    @Override
    public double getKA() {
        return 0.005;
    }

    @Override
    public double getMaxAngVel() {
        return Math.toRadians(2716.422) * SLOWDOWN;
    }

    @Override
    public double getLateralMultiplier() {
        return 4;
    }

    @Override
    public PIDCoefficients getHeadingPID() {
        return new PIDCoefficients(9, 0, 0);
    }

    @Override
    public PIDCoefficients getTranslationalPID() {
        return new PIDCoefficients(8, 0, 0);
    }

    @Override
    public double getEncoderLateralDistance() {
        return 13.25f;
    }

}
