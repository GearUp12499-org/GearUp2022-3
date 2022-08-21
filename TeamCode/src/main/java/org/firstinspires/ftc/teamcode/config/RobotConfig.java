package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.util.Encoder;

public interface RobotConfig {
    double getTicksPerRev();
    double getMaxRPM();
    boolean getRunUsingEncoder();
    PIDFCoefficients getModerVeloPID();
    double getWheelRadius();
    double getGearRatio();
    double getTrackWidth();
    double getKV();
    double getKA();
    double getKStatic();
    double getMaxVel();
    double getMaxAccel();
    double getMaxAngVel();
    double getMaxAngAccel();

    double getEncoderTicksPerRev();
    double getEncoderWheelRadius(); // in
    double getEncoderGearRatio(); // output (wheel) speed / input (encoder) speed
    double getEncoderLateralDistance(); // in; distance between the left and right wheels
    double getEncoderForwardOffset(); // in; offset of the lateral wheel

    default double getXMultiplier() { return 1; }
    default double getYMultiplier() { return 1; }

    interface MotorConfiguration {
        DcMotorEx getLeftFrontMotor(HardwareMap hardwareMap);
        DcMotorEx getLeftRearMotor(HardwareMap hardwareMap);
        DcMotorEx getRightFrontMotor(HardwareMap hardwareMap);
        DcMotorEx getRightRearMotor(HardwareMap hardwareMap);

        Encoder getLeftEncoder(HardwareMap hardwareMap);
        Encoder getRightEncoder(HardwareMap hardwareMap);
        Encoder getFrontEncoder(HardwareMap hardwareMap);
    }

    MotorConfiguration getMotorConfiguration();
    // TODO add motor + encoder direction configuration
    // TODO add hardware configuration (motors + odometry)

    default double rpmToVelocity(double rpm) {
        return rpm * getGearRatio() * 2 * Math.PI * getWheelRadius() / 60.0;
    }

    default double getMotorVelocityF(double ticksPerSecond) {
        // see https://docs.google.com/document/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/edit#heading=h.61g9ixenznbx
        return 32767 / ticksPerSecond;
    }
}
