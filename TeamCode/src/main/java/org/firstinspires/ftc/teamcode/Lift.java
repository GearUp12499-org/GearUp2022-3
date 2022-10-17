package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    private static final int UPPER_LIMIT = 3500;
    private static final int LOWER_LIMIT = 20;
    private final DcMotor l1;
    private final DcMotor l2;

    public static final double upSpeed = 0.5;
    public static final double downSpeed = -0.8;

    public Lift(HardwareMap hardwareMap) {
        l1 = hardwareMap.get(DcMotor.class, "lift1");
        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2 = hardwareMap.get(DcMotor.class, "lift2");
        l2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setDirection(DcMotorSimple.Direction.REVERSE);
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void stop() {
        l1.setPower(0);
        l2.setPower(0);
    }

    public void up() {
        if (unsafe(upSpeed)) return;
        l1.setPower(upSpeed);
        l2.setPower(upSpeed);
    }

    public void down() {
        if (unsafe(downSpeed)) return;
        l1.setPower(downSpeed);
        l2.setPower(downSpeed);
    }

    public int getEncoderCounts() {
        return (int) ((long)l1.getCurrentPosition() + l2.getCurrentPosition()) / 2;
    }

    public boolean unsafe(double speed) {
        if (getEncoderCounts() > UPPER_LIMIT && speed > 0) return true;
        else return getEncoderCounts() < LOWER_LIMIT && speed < 0;
    }

    public void safety() {
        if (unsafe(l1.getPower())) stop();
    }
}
