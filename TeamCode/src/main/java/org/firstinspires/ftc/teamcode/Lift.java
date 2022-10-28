package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public static final int[] TARGETS = {20, 1450, 2200, 3100};
    public static final int LOWER_BOUND = 20, UPPER_BOUND = 3500;
    public int currentTarget = 0, targetCount = TARGETS[0];
    public final DcMotor l1;
    public final DcMotor l2;

    public Lift(HardwareMap hardwareMap) {
        l1 = hardwareMap.get(DcMotor.class, "lift1");
        l1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l1.setPower(1);
        l1.setTargetPosition(TARGETS[currentTarget]);
        l1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        l1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        l2 = hardwareMap.get(DcMotor.class, "lift2");
        l2.setPower(0);
        l2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        l2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l2.setDirection(DcMotorSimple.Direction.REVERSE);
        l2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void update() {
        if (l1.getCurrentPosition() > targetCount + 20)
            l2.setPower(-1);
        else if (l1.getCurrentPosition() < targetCount - 20)
            l2.setPower(1);
        else
            l2.setPower(0);
    }

    public void updTarget() {
        targetCount = Math.max(targetCount, LOWER_BOUND);
        targetCount = Math.min(targetCount, UPPER_BOUND);
        l1.setTargetPosition(targetCount);
    }

    public void upTarget() {
        currentTarget = currentTarget == TARGETS.length - 1 ? currentTarget : currentTarget + 1;
        targetCount = TARGETS[currentTarget];
        updTarget();
    }

    public void downTarget() {
        currentTarget = currentTarget == 0 ? currentTarget : currentTarget - 1;
        targetCount = TARGETS[currentTarget];
        updTarget();
    }

    public void setTarget(int index) {
        currentTarget = index;
        targetCount = TARGETS[currentTarget];
        updTarget();
    }

    public void move(int delta) {
        targetCount += delta;
        updTarget();
    }
}
