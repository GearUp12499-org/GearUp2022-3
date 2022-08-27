package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SharedHardware {
    public static ElapsedTime runtime = new ElapsedTime();
    public static DcMotor frontLeft;
    public static DcMotor frontRight;
    public static DcMotor rearLeft;
    public static DcMotor rearRight;

    public static DcMotor encoderLeft;
    public static DcMotor encoderRight;
    public static DcMotor encoderRear;

    public static void prepareHardware(HardwareMap hardwareMap) {
        // Correct names as of July 9 2022
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearLeft = hardwareMap.get(DcMotor.class, "rear_left");
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rearRight = hardwareMap.get(DcMotor.class, "rear_right");
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //copy motors over to encoders
        encoderRight = frontRight;   // 0
        encoderLeft = frontLeft;   // 1
        encoderRear = rearRight;    // 2
    }
}
