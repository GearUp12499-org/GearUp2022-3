package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Lift {
    public static final int[] VERTICAL_TARGETS = {40, inEnc(14), inEnc(26), 4500};
    public static final int[] HORIZONTAL_TARGETS = {30, 220};
    public static final double[] HORIZONTAL_POWER_LEVEL = {0, 0.65, 0.8, 1};
    public static final int LOWER_VERTICAL_BOUND = 20, UPPER_VERTICAL_BOUND = 4600;  // 3500
    public static final int LOWER_HORIZONTAL_BOUND = 30, UPPER_HORIZONTAL_BOUND = 850; // was 225, then 500
    public int currentVerticalTarget = 0, targetVerticalCount = VERTICAL_TARGETS[0];

    public int getFakedVerticalCount() {
        return (int) (targetVerticalCount * (384.5 / 537.7));
    }

    public int currentHorizontalTarget = 0, targetHorizontalCount = HORIZONTAL_TARGETS[0];

    public boolean correcting = false;
    public boolean traveling = false;

    public final DcMotor liftVertical1;
    public final DcMotor liftVertical2;
    public final DcMotor liftHorizontal;

    public static final double POWER_UP = 1.0;
    public static final double POWER_CORRECT = 0.2;
    public static final double POWER_H = 0.65; //0.65
    public static final double POWER_DOWN = -0.8;

    public static final double ENCODER_COUNTS_PER_ROTATION = 537.7;
    public static final double SPOOL_DIAMETER = 1.25;
    public static final double SPOOL_CIRCUMFERENCE = SPOOL_DIAMETER * Math.PI;
    public static final double COUNTS_TO_INCHES_FACTOR = SPOOL_CIRCUMFERENCE / ENCODER_COUNTS_PER_ROTATION;

    public static final double START_CORRECTING = -20;
    public static final double STOP_CORRECTING = 0;

    public final Servo servo;

    private static int encIn(int counts) {
        return (int) (counts * COUNTS_TO_INCHES_FACTOR);
    }

    public static int inEnc(int inches) {
        return (int) (inches / COUNTS_TO_INCHES_FACTOR);
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    public void initLiftMotors() {
        liftVertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVertical1.setPower(POWER_UP);
        liftVertical1.setDirection(DcMotorSimple.Direction.REVERSE);
        liftVertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftVertical1.setPower(0);

        liftVertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftVertical2.setPower(0);

        liftHorizontal.setPower(POWER_H);
        liftHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftHorizontal.setTargetPosition(0);
        liftHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftHorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        liftHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Lift(HardwareMap hardwareMap) {
        liftVertical1 = hardwareMap.get(DcMotor.class, "lift1");

        liftVertical2 = hardwareMap.get(DcMotor.class, "lift2");

        liftHorizontal = hardwareMap.get(DcMotor.class, "liftHorizontal");
        initLiftMotors();
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(0.25);
    }

    /**
     * Check current state of the lift, update motor powers accordingly.
     */
    public void update() {
        if (!traveling) {
            // Correction
            double diff = liftVertical1.getCurrentPosition() - getFakedVerticalCount();
            if (diff < START_CORRECTING && !correcting) {
                liftVertical1.setPower(POWER_CORRECT);
                liftVertical2.setPower(POWER_CORRECT);
                correcting = true;
            } else if (diff > STOP_CORRECTING && correcting) {
                liftVertical1.setPower(0);
                liftVertical2.setPower(0);
                correcting = false;
            }
        } else {
            correcting = false;
            if (liftVertical1.getCurrentPosition() > getFakedVerticalCount() + 30) {
                liftVertical1.setPower(POWER_DOWN);
                liftVertical2.setPower(POWER_DOWN);
            } else if (liftVertical1.getCurrentPosition() < getFakedVerticalCount() - 30 && liftVertical1.getCurrentPosition() > getFakedVerticalCount() - 200) {
                liftVertical1.setPower(0.4);
                liftVertical2.setPower(0.4);
            } else if (liftVertical1.getCurrentPosition() < getFakedVerticalCount() - 30) {
                liftVertical1.setPower(POWER_UP);
                liftVertical2.setPower(POWER_UP);
            }
            else {
                liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                liftVertical1.setPower(0);
                liftVertical2.setPower(0);
                traveling = false;
            }
        }
    }

    public void updTargets() {
        targetVerticalCount = clamp(targetVerticalCount, LOWER_VERTICAL_BOUND, UPPER_VERTICAL_BOUND);
        targetHorizontalCount = clamp(targetHorizontalCount, LOWER_HORIZONTAL_BOUND, UPPER_HORIZONTAL_BOUND);
        liftHorizontal.setTargetPosition(targetHorizontalCount);
        traveling = true;
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
     *
     * @param position Target encoder position.
     * @param op       OpMode to use for thread control. (stopping) (can be null)
     * @throws InterruptedException If the thread is interrupted or the op-mode is stopped.
     */
    public void verticalLift(int position, @Nullable LinearOpMode op) throws InterruptedException { //runs without encoder
        setVerticalTargetManual(position);
        waitLift(op);
    }

    /**
     * Wait for the lifts to reach their target positions.
     *
     * @param op OpMode to use for thread control. (stopping) (can be null)
     * @throws InterruptedException If the thread is interrupted or the op-mode is stopped.
     */
    public void waitLift(@Nullable LinearOpMode op) throws InterruptedException {
        while (!(isSatisfiedHorizontally() && isSatisfiedHorizontally() && (op == null || op.opModeIsActive()))) {
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
        servo.setPosition(0.62);
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
        final int MAX_FUDGERY = 80;
        return Math.abs(targetHorizontalCount - liftHorizontal.getCurrentPosition()) < MAX_FUDGERY;
    }

    public void setHorizontalPower(int index) {
        liftHorizontal.setPower(HORIZONTAL_POWER_LEVEL[index]);
    }
}