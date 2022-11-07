package org.firstinspires.ftc.teamcode.nav;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.NotImplemented;

import java.util.ArrayList;
import java.util.List;

public class EncoderNavigation {
    public static class Action {
        public Action(Type type, double value) {
            this.type = type;
            this.value = value;
        }

        public enum Type {
            FORWARD
        }
        public final Type type;
        public final double value;
    }

    public List<Action> actionQueue = new ArrayList<>();

    public Action currentAction;
    public int startEncoderValue;
    public int lastKnownProgress;
    public int targetEncoderValue;

    public final DcMotor leftFront;
    public final DcMotor rightFront;
    public final DcMotor leftRear;
    public final DcMotor rightRear;

    public final Encoder leftOdom;
    public final Encoder rightOdom;
    public final Encoder frontOdom;

    public double topSpeed = 0.5;

    /**
     * inches, distance to start ramping down speed
     */
    public double rampDownDistance = 3;

    /**
     * inches, distance to consider being "done"
     */
    public double fudge = 0.2;

    public static final int TICKS_PER_REV = 8192;
    public static final double ODOM_WHEEL_RADIUS = 0.69;
    public static final double TICKS_PER_INCH = TICKS_PER_REV / (ODOM_WHEEL_RADIUS * 2 * PI);

    public EncoderNavigation(
            DcMotor leftFront, DcMotor rightFront, DcMotor leftRear, DcMotor rightRear,
            Encoder leftOdom, Encoder rightOdom, Encoder frontOdom
    ) {
        this.leftFront = leftFront;
        this.rightFront = rightFront;
        this.leftRear = leftRear;
        this.rightRear = rightRear;
        this.leftOdom = leftOdom;
        this.rightOdom = rightOdom;
        this.frontOdom = frontOdom;
    }

    private int forwardValue() {
        return (leftOdom.getCurrentPosition() + rightOdom.getCurrentPosition()) / 2;
    }

    public double toInches(int ticks) {
        return ticks / TICKS_PER_INCH;
    }

    public double toTicks(double inches) {
        return inches * TICKS_PER_INCH;
    }

    /**
     * Add a task to make the robot move forward.
     * @param inches inches to travel forward
     * @param override if true, will override any existing tasks
     */
    public void moveForward(double inches, boolean override) {
        if (override) actionQueue.clear();
        actionQueue.add(new Action(Action.Type.FORWARD, inches));
    }

    /**
     * Add a task to make the robot move forward.
     * @param inches inches to travel forward
     */
    public void moveForward(double inches) {
        moveForward(inches, false);
    }

    public void setGroup(double power, DcMotor ... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void asyncLoop() {
        if (currentAction == null) {
            if (actionQueue.size() > 0) {
                currentAction = actionQueue.remove(0);
                switch (currentAction.type) {
                    case FORWARD:
                        startEncoderValue = forwardValue();
                        targetEncoderValue = startEncoderValue + (int) (currentAction.value * TICKS_PER_INCH);
                        break;
                    default:
                        throw NotImplemented.i;
                }
            }
        }
        if (currentAction != null) {
            int front = frontOdom.getCurrentPosition();
            int left = leftOdom.getCurrentPosition();
            int right = rightOdom.getCurrentPosition();
            int fwd = forwardValue();

            switch (currentAction.type) {
                case FORWARD:
                    int distToTarget = targetEncoderValue - fwd;
                    lastKnownProgress = fwd;
                    double sign = Math.signum(distToTarget);
                    distToTarget = abs(distToTarget);
                    double fixed = topSpeed * sign;
                    double speed = toInches(distToTarget) > rampDownDistance ? fixed : fixed * (toInches(distToTarget) / rampDownDistance);

                    setGroup(speed, leftFront, rightFront, leftRear, rightRear);

                    if (Math.abs(distToTarget) < toTicks(fudge)) {
                        currentAction = null;
                        setGroup(0, leftFront, rightFront, leftRear, rightRear);
                    }
                    break;
                default:
                    throw NotImplemented.i;
            }
        }
    }
}
