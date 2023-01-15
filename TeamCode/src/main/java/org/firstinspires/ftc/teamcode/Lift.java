package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public static final int[] VERTICAL_TARGETS = {20, 1450, 2200, 3100};
    public static final int[] HORIZONTAL_TARGETS = {40, 220};
    public static final int LOWER_VERTICAL_BOUND = 20, UPPER_VERTICAL_BOUND = 3500;
    public static final int LOWER_HORIZONTAL_BOUND = 0, UPPER_HORIZONTAL_BOUND = 850; // was 225, then 500
    public int currentVerticalTarget = 0, targetVerticalCount = VERTICAL_TARGETS[0];
    public int currentHorizontalTarget = 0, targetHorizontalCount = HORIZONTAL_TARGETS[0];
    public final DcMotor liftVertical1;
    public final DcMotor liftVertical2;
    public final DcMotor liftHorizontal;

    public static final double POWER_UP = 1.0;
    public static final double POWER_H = 0.65; //0.65
    public static final double POWER_DOWN = -0.8;

    public static final double ENCODER_COUNTS_PER_ROTATION = 537.7;
    public static final double SPOOL_DIAMETER = 1.25;
    public static final double SPOOL_CIRCUMFERENCE = SPOOL_DIAMETER * Math.PI;
    public static final double COUNTS_TO_INCHES_FACTOR = SPOOL_CIRCUMFERENCE / ENCODER_COUNTS_PER_ROTATION;

    public final Servo servo;

    private static int encoderCountsToInches(int counts) {
        return (int)(counts * COUNTS_TO_INCHES_FACTOR);
    }
    private static int inchesToEncoderCounts(int inches) {
        return (int)(inches / COUNTS_TO_INCHES_FACTOR);
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    public Lift(HardwareMap hardwareMap) {
        liftVertical1 = hardwareMap.get(DcMotor.class, "lift1");
        liftVertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVertical1.setPower(POWER_UP);
        liftVertical1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftVertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftVertical1.setPower(0);

        liftVertical2 = hardwareMap.get(DcMotor.class, "lift2");
        liftVertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftVertical2.setPower(0);

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
        if (liftVertical1.getCurrentPosition() > targetVerticalCount + 20) {
            liftVertical1.setPower(POWER_DOWN);
            liftVertical2.setPower(POWER_DOWN);
        } else if (liftVertical1.getCurrentPosition() < targetVerticalCount - 20) {
            liftVertical1.setPower(POWER_UP);
            liftVertical2.setPower(POWER_UP);
        } else {
            liftVertical1.setPower(0);
            liftVertical2.setPower(0);
        }
    }

    public void updTargets() {
        targetVerticalCount = clamp(targetVerticalCount, LOWER_VERTICAL_BOUND, UPPER_VERTICAL_BOUND);
        targetHorizontalCount = clamp(targetHorizontalCount, LOWER_HORIZONTAL_BOUND, UPPER_HORIZONTAL_BOUND);
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

    /**
     * Move the vertical lift to a specific position (blocking).
     * @param position Target encoder position.
     * @param op OpMode to use for thread control. (stopping) (can be null)
     * @throws InterruptedException If the thread is interrupted or the op-mode is stopped.
     */
    public void verticalLift(int position, @Nullable LinearOpMode op) throws InterruptedException { //runs without encoder
        setVerticalTargetManual(position);
        waitLift(op);
    }

    /**
     * Wait for the lifts to reach their target positions.
     * @param op OpMode to use for thread control. (stopping) (can be null)
     * @throws InterruptedException If the thread is interrupted or the op-mode is stopped.
     */
    public void waitLift(@Nullable LinearOpMode op) throws InterruptedException {
        while(!(isSatisfiedHorizontally() && isSatisfiedHorizontally() && (op == null || op.opModeIsActive()))) {
            update();
            // spin :(
            Thread.sleep(10);
        }
        update();
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