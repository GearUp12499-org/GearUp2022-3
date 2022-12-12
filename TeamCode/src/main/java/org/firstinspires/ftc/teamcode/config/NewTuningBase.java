package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.DriveData.currentMotorConfiguration;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class NewTuningBase implements RobotConfig {
    @Override
    public double getTicksPerRev() {
        return 537.6;
    }

    @Override
    public double getMaxRPM() {
        return 312;
    }

    @Override
    public boolean getRunUsingEncoder() {
        return false;
    }

    @Override
    public PIDFCoefficients getMotorVeloPID() {
        return new PIDFCoefficients(0, 0, 0,
                getMotorVelocityF(getMaxRPM() / 60 * getTicksPerRev()));
    }

    @Override
    public double getWheelRadius() {
        return 1.88976;
    }

    @Override
    public double getGearRatio() {
        return 1; // TODO Check me
    }

    @Override
    public double getTrackWidth() {
        return 0; // TODO Check me
    }

    @Override
    public double getKV() {
        return 1.0 / rpmToVelocity(getMaxRPM());
    }

    @Override
    public double getKA() {
        return 0;
    }

    @Override
    public double getKStatic() {
        return 0;
    }

    /* TODO Fill in these two values.
     * Note from LearnRoadRunner.com:
     * The velocity and acceleration constraints were calculated based on the following equation:
     * ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * Math.PI) * 0.85
     * Resulting in 52.48180821614297 in/s.
     * This is only 85% of the theoretical maximum velocity of the bot, following the recommendation above.
     * This is capped at 85% because there are a number of variables that will prevent your bot from actually
     * reaching this maximum velocity: voltage dropping over the game, bot weight, general mechanical inefficiencies, etc.
     * However, you can push this higher yourself if you'd like. Perhaps raise it to 90-95% of the theoretically
     * max velocity. The theoretically maximum velocity is 61.74330378369762 in/s.
     * Just make sure that your bot can actually reach this maximum velocity. Path following will be detrimentally
     * affected if it is aiming for a velocity not actually possible.
     *
     * The maximum acceleration is somewhat arbitrary and it is recommended that you tweak this yourself based on
     * actual testing. Just set it at a reasonable value and keep increasing until your path following starts
     * to degrade. As of now, it simply mirrors the velocity, resulting in 52.48180821614297 in/s/s
     *
     * Maximum Angular Velocity is calculated as: maximum velocity / trackWidth * (180 / Math.PI) but capped at 360°/s.
     * You are free to raise this on your own if you would like. It is best determined through experimentation.

     */
    @Override
    public double getMaxVel() {
        return 0; // TODO
    }

    @Override
    public double getMaxAccel() {
        return 0; // TODO
    }

    @Override
    public double getMaxAngVel() {
        return 0; // TODO
    }

    @Override
    public double getMaxAngAccel() {
        return 0; // TODO
    }

    @Override
    public double getEncoderTicksPerRev() {
        return 8192;
    }

    @Override
    public double getEncoderWheelRadius() {
        return 0.69; // TODO check me
    }

    @Override
    public double getEncoderGearRatio() {
        return 1;
    }

    @Override
    public double getEncoderLateralDistance() {
        return 0; // TODO check me
    }

    @Override
    public double getEncoderForwardOffset() {
        return 0; // TODO check me; should be NEGATIVE
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
    public MotorConfiguration getMotorConfiguration() {
        return currentMotorConfiguration;
    }
}
