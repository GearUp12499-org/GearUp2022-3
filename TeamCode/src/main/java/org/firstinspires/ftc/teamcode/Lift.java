package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public static final int[] VERTICAL_TARGETS = {20, 1450, 2200, 3100};
    public static final int[] HORIZONTAL_TARGETS = {0, 220};
    public static final int LOWER_VERTICAL_BOUND = 20, UPPER_VERTICAL_BOUND = 3500;
    public static final int LOWER_HORIZONTAL_BOUND = 0, UPPER_HORIZONTAL_BOUND = 850; // was 225, then 500
    public int currentVerticalTarget = 0, targetVerticalCount = VERTICAL_TARGETS[0];
    public int currentHorizontalTarget = 0, targetHorizontalCount = HORIZONTAL_TARGETS[0];
    public final DcMotor liftVertical1;
    public final DcMotor liftVertical2;
    public final DcMotor liftHorizontal;

    public static final double POWER_UP = 1.0;
    public static final double POWER_H = 0.65;
    public static final double POWER_DOWN = -0.5;

    public final Servo servo;
    
    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    public Lift(HardwareMap hardwareMap) {
        liftVertical1 = hardwareMap.get(DcMotor.class, "lift1");
        liftVertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVertical1.setPower(POWER_UP);
        liftVertical1.setTargetPosition(0);
        liftVertical1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftVertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftVertical2 = hardwareMap.get(DcMotor.class, "lift2");
        liftVertical2.setPower(0);
        liftVertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftHorizontal = hardwareMap.get(DcMotor.class, "liftHorizontal");
        liftHorizontal.setPower(POWER_H);
        liftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftHorizontal.setTargetPosition(0);
        liftHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        liftHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.25);
    }

    public void update() {
        if (liftVertical1.getCurrentPosition() > targetVerticalCount + 20)
            liftVertical2.setPower(POWER_DOWN);
        else if (liftVertical1.getCurrentPosition() < targetVerticalCount - 20)
            liftVertical2.setPower(POWER_UP);
        else
            liftVertical2.setPower(0);
        if (liftVertical1.getCurrentPosition() > targetVerticalCount)
            liftVertical1.setPower(POWER_DOWN);
        else if (liftVertical1.getCurrentPosition() < targetVerticalCount)
            liftVertical1.setPower(POWER_UP);
    }

    public void updTargets() {
        targetVerticalCount = clamp(targetVerticalCount, LOWER_VERTICAL_BOUND, UPPER_VERTICAL_BOUND);
        targetHorizontalCount = clamp(targetHorizontalCount, LOWER_HORIZONTAL_BOUND, UPPER_HORIZONTAL_BOUND);
        liftVertical1.setTargetPosition(targetVerticalCount);
        liftHorizontal.setTargetPosition(targetHorizontalCount);
    }

    public void goUp() {
        currentVerticalTarget = currentVerticalTarget == VERTICAL_TARGETS.length - 1 ? currentVerticalTarget : currentVerticalTarget + 1;
        targetVerticalCount = VERTICAL_TARGETS[currentVerticalTarget];
        updTargets();
    }

    public void goDown() {
        currentVerticalTarget = currentVerticalTarget == 0 ? currentVerticalTarget : currentVerticalTarget - 1;
        targetVerticalCount = VERTICAL_TARGETS[currentVerticalTarget];
        updTargets();
    }

    public void extend() {
        currentHorizontalTarget = currentHorizontalTarget == HORIZONTAL_TARGETS.length - 1 ? currentHorizontalTarget : currentHorizontalTarget + 1;
        targetHorizontalCount = HORIZONTAL_TARGETS[currentHorizontalTarget];
        updTargets();
    }

    public void retract() {
        currentHorizontalTarget = currentHorizontalTarget == 0 ? currentHorizontalTarget : currentHorizontalTarget - 1;
        targetHorizontalCount = HORIZONTAL_TARGETS[currentHorizontalTarget];
        updTargets();
    }

    public void speedVlift(double counts) {
        while (liftVertical1.getCurrentPosition() < counts){
            liftVertical1.setPower(1);
            liftVertical2.setPower(1);
        }
        liftVertical1.setPower(0);
        liftVertical2.setPower(0);
    }
    public void setVerticalTarget(int index) {
        currentVerticalTarget = index;
        targetVerticalCount = VERTICAL_TARGETS[currentVerticalTarget];
        updTargets();
    }

    public void setHorizontalTarget(int index) {
        currentHorizontalTarget = index;
        targetHorizontalCount = HORIZONTAL_TARGETS[currentHorizontalTarget];
        updTargets();
    }

    public void setVerticalTargetManual(int target) {
        targetVerticalCount = target;
        updTargets();
    }

    public void setHorizontalTargetManual(int target) {
        targetHorizontalCount = target;
        updTargets();
    }

    public void moveVertical(int delta) {
        targetVerticalCount += delta;
        updTargets();
    }

    public void moveHorizontal(int delta) {
        targetHorizontalCount += delta;
        updTargets();
    }

    public void closeClaw() {
        servo.setPosition(0.52);
    }

    public void openClaw() {
        servo.setPosition(0.25);
    }

    public boolean isExtended() {
        return (currentVerticalTarget != LOWER_VERTICAL_BOUND || liftVertical1.getPower() == 0);
    }

    public boolean isSatisfiedVertically() {
        final int MAX_FUDGERY = 100;
        return Math.abs(targetVerticalCount - liftVertical1.getCurrentPosition()) < MAX_FUDGERY;
    }

    public boolean isSatisfiedHorizontally() {
        final int MAX_FUDGERY = 100;
        return Math.abs(targetHorizontalCount - liftHorizontal.getCurrentPosition()) < MAX_FUDGERY;
    }
}
