package org.firstinspires.ftc.teamcode.config;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Encoder;

/**
 * Robot configuration template base.
 * Use the I↓ or O↓ action icons (in the gutter) to go to a configuration class and change values.
 * COMMENTS are the file that the values below are used in.
 */
public interface RobotConfig {
    // DriveConstants
    double getTicksPerRev();
    double getMaxRPM();
    boolean getRunUsingEncoder();
    PIDFCoefficients getMotorVeloPID();
    double getWheelRadius();
    double getGearRatio();
    double getTrackWidth();
    double getKV();       // Feedforward tuning
    double getKA();       // Feedforward tuning
    double getKStatic();  // Feedforward tuning
    double getMaxVel();
    double getMaxAccel();
    double getMaxAngVel();
    double getMaxAngAccel();

    // StandardTrackingWheelLocalizer
    double getEncoderTicksPerRev();
    double getEncoderWheelRadius(); // in
    double getEncoderGearRatio(); // output (wheel) speed / input (encoder) speed
    double getEncoderLateralDistance(); // in; distance between the left and right wheels
    double getEncoderForwardOffset(); // in; offset of the lateral wheel
    default double getXMultiplier() { return 1; }
    default double getYMultiplier() { return 1; }

    // SampleMecanumDrive
    default double getLateralMultiplier() { return 1; }
    PIDCoefficients getHeadingPID();
    PIDCoefficients getTranslationalPID();


    interface MotorConfiguration {
        // SampleMecanumDrive
        @Nullable
        DcMotorEx getLeftFrontMotor();
        DcMotorEx getLeftFrontMotor(HardwareMap hardwareMap);
        @Nullable
        DcMotorEx getLeftRearMotor();
        DcMotorEx getLeftRearMotor(HardwareMap hardwareMap);
        @Nullable
        DcMotorEx getRightFrontMotor();
        DcMotorEx getRightFrontMotor(HardwareMap hardwareMap);
        @Nullable
        DcMotorEx getRightRearMotor();
        DcMotorEx getRightRearMotor(HardwareMap hardwareMap);

        // StandardTrackingWheelLocalizer
        @Nullable
        Encoder getLeftEncoder();
        Encoder getLeftEncoder(HardwareMap hardwareMap);
        @Nullable
        Encoder getRightEncoder();
        Encoder getRightEncoder(HardwareMap hardwareMap);
        @Nullable
        Encoder getFrontEncoder();
        Encoder getFrontEncoder(HardwareMap hardwareMap);
    }

    MotorConfiguration getMotorConfiguration();

    default double rpmToVelocity(double rpm) {
        return rpm * getGearRatio() * 2 * Math.PI * getWheelRadius() / 60.0;
    }

    default double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
