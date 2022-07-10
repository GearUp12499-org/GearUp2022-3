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

    public static void prepareHardware(HardwareMap hardwareMap) {
        // Correct names as of July 9 2022
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left");
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight = hardwareMap.get(DcMotor.class, "rear_right");
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
