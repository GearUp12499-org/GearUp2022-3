package org.firstinspires.ftc.teamcode.nav;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.NotImplemented;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class EncoderNavigation {
    public static class Action {
        public Action(Type type, double value) {
            this.type = type;
            this.value = value;
        }

        public enum Type {
            FORWARD, STRAFE
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

    public double topSpeed = 0.3;

    /**
     * inches, distance to start ramping down speed
     */
    public double rampDownDistance = 1;

    /**
     * Seconds, delay after "reaching" the goal to continue to check & move
     */
    public double holdForCompletion = 1; // TODO pull down to 0.5 once done testing

    private double completedFor = 0;

    private double lastTickTime = System.currentTimeMillis() / 1000.0;

    public double getWaitPercent() {
        return completedFor / holdForCompletion;
    }

    public double strafeFrontFudge = 1;
    public double strafeRearFudge = 1;

    public double forwardLeftFudge = 1;
    public double forwardRightFudge = 1;

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

    public boolean isDone() {
        return actionQueue.isEmpty();
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

    /**
     * Add a task to make the robot strafe right.
     * @param inches inches to strafe right
     * @param override if true, will override any existing tasks
     */
    public void strafeRight(double inches, boolean override) {
        if (override) actionQueue.clear();
        actionQueue.add(new Action(Action.Type.STRAFE, inches));
    }

    /**
     * Add a task to make the robot strafe right.
     * @param inches inches to strafe right
     */
    public void strafeRight(double inches) {
        strafeRight(inches, false);
    }

    /**
     * Add a task to make the robot strafe left.
     * @param inches inches to strafe left
     * @param override if true, will override any existing tasks
     */
    public void strafeLeft(double inches, boolean override) {
        strafeRight(-inches, override);
    }

    /**
     * Add a task to make the robot strafe left.
     * @param inches inches to strafe left
     */
    public void strafeLeft(double inches) {
        strafeLeft(inches, false);
    }

    public void setGroup(double power, DcMotor ... motors) {
        for (DcMotor motor : motors) {
            motor.setPower(power);
        }
    }

    public void asyncLoop() {
        double thisTickTime = System.currentTimeMillis() / 1000.0;
        if (currentAction == null) {
            if (actionQueue.size() > 0) {
                currentAction = actionQueue.remove(0);
                switch (currentAction.type) {
                    case FORWARD:
                        startEncoderValue = forwardValue();
                        targetEncoderValue = startEncoderValue + (int) (currentAction.value * TICKS_PER_INCH);
                        break;
                    case STRAFE:
                        startEncoderValue = frontOdom.getCurrentPosition();
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
            int distToTarget;
            double sign;
            double fixed;
            double speed;

            if (completedFor >= holdForCompletion) {
                currentAction = null;
                completedFor = 0;
                setGroup(0, leftFront, rightFront, leftRear, rightRear);
                return;
            }

            switch (currentAction.type) {
                case FORWARD:
                    distToTarget = targetEncoderValue - fwd;
                    lastKnownProgress = fwd;
                    sign = Math.signum(distToTarget);
                    distToTarget = abs(distToTarget);
                    fixed = topSpeed * sign;
                    speed = toInches(distToTarget) > rampDownDistance ? fixed : fixed * (toInches(distToTarget) / rampDownDistance);

                    setGroup(speed * forwardLeftFudge, leftFront, leftRear);
                    setGroup(speed * forwardRightFudge, rightFront, rightRear);

                    if (Math.abs(distToTarget) < toTicks(fudge)) {
                        completedFor += thisTickTime - lastTickTime;
                    } else {
                        completedFor = 0;
                    }
                    break;
                case STRAFE:
                    distToTarget = targetEncoderValue - front;
                    lastKnownProgress = front;
                    sign = Math.signum(distToTarget);
                    distToTarget = abs(distToTarget);
                    fixed = topSpeed * sign;
                    speed = toInches(distToTarget) > rampDownDistance ? fixed : fixed * (toInches(distToTarget) / rampDownDistance);

                    setGroup(speed * strafeFrontFudge, leftFront);
                    setGroup(speed * strafeRearFudge, rightRear);
                    setGroup(-speed * strafeFrontFudge, rightFront);
                    setGroup(-speed * strafeRearFudge, leftRear);

                    if (Math.abs(distToTarget) < toTicks(fudge)) {
                        completedFor += thisTickTime - lastTickTime;
                    } else {
                        completedFor = 0;
                    }
                    break;
                default:
                    throw NotImplemented.i;
            }
        }
        lastTickTime = thisTickTime;
    }

    public void dumpTelemetry(Telemetry telemetry) {
        telemetry.addLine("== Navigation ==");
        telemetry.addData("left", leftOdom.getCurrentPosition());
        telemetry.addData("right", rightOdom.getCurrentPosition());
        telemetry.addData("center", frontOdom.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("completion %", Math.round(lastKnownProgress / (double) targetEncoderValue * 10000)/100);
        if (getWaitPercent() > 0) {
            telemetry.addData("delay %", Math.round(getWaitPercent()*10000)/100);
        }
        telemetry.addLine();
        telemetry.addData("queue count", actionQueue.size());
        telemetry.addLine("== End Navigation ==");
    }
}
