package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    public static final int[] TARGETS = {20, 1000, 2250, 3500};
    public int currentTarget = 0;
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
        l2.setPower(l1.getPower());
    }

    public void updTarget() {
        l1.setTargetPosition(TARGETS[currentTarget]);
    }

    public void up() {
        currentTarget = currentTarget == TARGETS.length - 1 ? currentTarget : currentTarget + 1;
        updTarget();
    }

    public void down() {
        currentTarget = currentTarget == 0 ? currentTarget : currentTarget - 1;
        updTarget();
    }
}
