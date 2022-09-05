package org.firstinspires.ftc.teamcode.config;


import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class GoBuildaV5MotorConf implements RobotConfig.MotorConfiguration {
    private DcMotorEx leftFront = null;
    private DcMotorEx leftRear = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightRear = null;

    public Encoder leftEncoder = null;
    public Encoder rightEncoder = null;
    public Encoder frontEncoder = null;

    public static final String LEFT_FRONT_MOTOR_NAME = "front_left";
    public static final String LEFT_REAR_MOTOR_NAME = "rear_left";
    public static final String RIGHT_FRONT_MOTOR_NAME = "front_right";
    public static final String RIGHT_REAR_MOTOR_NAME = "rear_right";

    public static final String LEFT_ENCODER_NAME = "front_left";
    public static final String RIGHT_ENCODER_NAME = "front_right";
    public static final String FRONT_ENCODER_NAME = "rear_right";

    @Nullable
    @Override
    public DcMotorEx getLeftFrontMotor() {
        return leftFront;
    }

    @Override
    public DcMotorEx getLeftFrontMotor(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, LEFT_FRONT_MOTOR_NAME);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        return leftFront;
    }

    @Nullable
    @Override
    public DcMotorEx getLeftRearMotor() {
        return leftRear;
    }

    @Override
    public DcMotorEx getLeftRearMotor(HardwareMap hardwareMap) {
        leftRear = hardwareMap.get(DcMotorEx.class, LEFT_REAR_MOTOR_NAME);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        return leftRear;
    }

    @Nullable
    @Override
    public DcMotorEx getRightFrontMotor() {
        return rightFront;
    }

    @Override
    public DcMotorEx getRightFrontMotor(HardwareMap hardwareMap) {
        rightFront = hardwareMap.get(DcMotorEx.class, RIGHT_FRONT_MOTOR_NAME);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        return rightFront;
    }

    @Nullable
    @Override
    public DcMotorEx getRightRearMotor() {
        return rightRear;
    }

    @Override
    public DcMotorEx getRightRearMotor(HardwareMap hardwareMap) {
        rightRear = hardwareMap.get(DcMotorEx.class, RIGHT_REAR_MOTOR_NAME);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        return rightRear;
    }

    @Nullable
    @Override
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    @Override
    public Encoder getLeftEncoder(HardwareMap hardwareMap) {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, LEFT_ENCODER_NAME));
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        return leftEncoder;
    }

    @Nullable
    @Override
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    @Override
    public Encoder getRightEncoder(HardwareMap hardwareMap) {
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, RIGHT_ENCODER_NAME));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        return rightEncoder;
    }

    @Nullable
    @Override
    public Encoder getFrontEncoder() {
        return frontEncoder;
    }

    @Override
    public Encoder getFrontEncoder(HardwareMap hardwareMap) {
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, FRONT_ENCODER_NAME));
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        return frontEncoder;
    }
}
