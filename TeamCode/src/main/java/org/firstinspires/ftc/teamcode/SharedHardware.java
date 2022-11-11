package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

public class SharedHardware {
    public static ElapsedTime runtime = new ElapsedTime();
    public static DcMotorEx frontLeft;
    public static DcMotorEx frontRight;
    public static DcMotorEx rearLeft;
    public static DcMotorEx rearRight;

    public static DcMotor turret;

    public static Encoder encoderLeft;
    public static Encoder encoderRight;
    public static Encoder encoderRear;

    public static void prepareHardware(HardwareMap hardwareMap) {
        // Correct names as of Oct 23 2022
        try {
            frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (IllegalArgumentException ignore) {
        }

        try {
            frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException ignore) {
        }

        try {
            rearLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
            rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (IllegalArgumentException ignore) {
        }
        try {
            rearRight = hardwareMap.get(DcMotorEx.class, "rightBack");
            rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
            rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException ignore) {
        }

        try {
            turret = hardwareMap.get(DcMotor.class, "turret");
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException ignore) {
        }

        //copy motors over to encoders
        encoderRight = new Encoder(frontRight);   // 0
        encoderLeft = new Encoder(frontLeft);   // 1
        encoderRear = new Encoder(rearRight);    // 2
    }
}
