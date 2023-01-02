package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Consumer;

public class DetectPoleV2 {
    public static int realMMToEncoderH(double mms) {
        // 220 et => 160 mm
        //
        return (int) ((16/22.0) * mms);
    }

    public static double readMMtoRealMM(double reading) {
        return reading - 0;
    }

    private static final int MAX_SCAN_FROM_CENTER = 500;
    /**
     * DO NOT:
     * <ul>
     *  <li>call changeState(...) every loop</li>
     *  <li>use State.DONE to request that the scanner drop control of the turret</li>
     *  <li>spam State.IDLE to stop the turret (just turn off the power manually)</li>
     * </ul>
     */
    public final DcMotor turret;
    public final DistanceSensor distanceSensor;
    public final Lift liftController;
    public final boolean extraActions;
    public double lastDistance;
    public double captureDistance;

    public State getState() {
        return currentState;
    }

    public void beginScan(RotateDirection direction) {
        if (currentState != State.IDLE) return;
        this.rotateDirection = direction;
        stateChange(State.BEGIN);
    }

    public enum State {
        /* Result states */
        IDLE,
        WAITING, /* USED INTERNALLY DO NOT DELETE */
        BEGIN,
        DONE,

        /* Action states; first to last */
        LIFT_UP1,
        ROTATE1, /* This is the only one that runs with extraActions == false */
        LIFT_UP2,
        EXTEND,
        CLAW_OPEN,
        RETRACT,
        LIFT_DOWN
    }

    public static final Map<State, Consumer<DetectPoleV2>> ON_ENTER_DEFAULTS = new HashMap<>();
    public static final Map<State, Consumer<DetectPoleV2>> ON_EXIT_DEFAULTS = new HashMap<>();

    public static final double SPEED = 0.35;

    // Static class initializer; runs once when the class is loaded
    static {   // Debugging? This block is called <clinit>
        ON_ENTER_DEFAULTS.put(State.IDLE, o -> {
            o.turret.setPower(0); // Force stop
        });
        ON_ENTER_DEFAULTS.put(State.DONE, o -> {
            o.turret.setPower(0); // Stop as well
        });
        ON_ENTER_DEFAULTS.put(State.BEGIN, o -> o.stateChange(o.extraActions ? State.LIFT_UP1 : State.ROTATE1));  // Change entrypoint here

        ON_ENTER_DEFAULTS.put(State.LIFT_UP1, o -> o.liftController.setVerticalTarget(1));

        ON_ENTER_DEFAULTS.put(State.ROTATE1, o -> {
            o.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            o.turret.setPower(SPEED * o.rotateDirection.powerModifier);
        });

        ON_EXIT_DEFAULTS.put(State.ROTATE1, o -> o.captureDistance = o.lastDistance);

        ON_ENTER_DEFAULTS.put(State.LIFT_UP2, o -> o.liftController.setVerticalTarget(3));

        ON_ENTER_DEFAULTS.put(State.EXTEND, o -> {
            o.liftController.setHorizontalTargetManual(realMMToEncoderH(readMMtoRealMM(o.captureDistance)));
        });

        ON_ENTER_DEFAULTS.put(State.CLAW_OPEN, o -> {
            o.liftController.openClaw();
        });

        ON_ENTER_DEFAULTS.put(State.RETRACT, o -> {
            o.liftController.setHorizontalTarget(0);
        });

        ON_ENTER_DEFAULTS.put(State.LIFT_DOWN, o -> o.liftController.setVerticalTarget(1));
    }

    private final Map<State, Consumer<DetectPoleV2>> onEnter = new HashMap<>(ON_ENTER_DEFAULTS);
    private final Map<State, Consumer<DetectPoleV2>> onExit = new HashMap<>(ON_EXIT_DEFAULTS);

    public DetectPoleV2(DcMotor turret, DistanceSensor distanceSensor, Lift liftController, boolean extraActions) {
        this.turret = turret;
        this.distanceSensor = distanceSensor;
        this.liftController = liftController;
        this.extraActions = extraActions;
    }

    // <init> aliases
    public DetectPoleV2(DcMotor turret, DistanceSensor distanceSensor, Lift liftController) {
        this(turret, distanceSensor, liftController, false);
    }

    public void dropControls() {
        if (currentState == State.IDLE) {
            return;
        }
        stateChange(State.IDLE);
    }

    public void attachOnEnter(State state, Consumer<DetectPoleV2> runnable) {
        onEnter.put(state, runnable);
    }

    public void attachOnExit(State state, Consumer<DetectPoleV2> runnable) {
        onExit.put(state, runnable);
    }

    public enum RotateDirection {
        CW(-1),
        CCW(1);

        public final int powerModifier;
        RotateDirection(int powerModifier) {
            this.powerModifier = powerModifier;
        }
    }

    final class DelayStateChange {
        private final double seconds;
        private final ElapsedTime timer;
        private final State to;
        public boolean expired = false;

        public DelayStateChange(double seconds, State to) {
            this.seconds = seconds;
            this.to = to;
            this.timer = new ElapsedTime();

            // Mock partial state change
            stateChange(State.WAITING);
        }

        public void update() {
            if (timer.seconds() > seconds) {
                expired = true;
                currentState = to;

                // Mock partial state change
                stateChange(to);
            }
        }
    }

    private State currentState = State.IDLE;

    public void stateChange(State newState) {
        Consumer<DetectPoleV2> exit = onExit.get(currentState);
        if (exit == null) exit = ign -> {};
        exit.accept(this);
        currentState = newState;
        Consumer<DetectPoleV2> enter = onEnter.get(currentState);
        if (enter == null) enter = ign -> {};
        enter.accept(this);
    }

    private DelayStateChange delay = null;

    public RotateDirection rotateDirection = RotateDirection.CW;

    public void run() {
        if (currentState == State.IDLE) {
            // do nothing
            return;
        }
        if (delay != null) {
            // delay is active, do not run
            delay.update();
            if (delay.expired) {
                delay = null;
            }
            return;
        }
        switch (currentState) {
            case ROTATE1:
                // check distance
                lastDistance = distanceSensor.getDistance(DistanceUnit.MM);

                if (lastDistance < 500) {
                    // stop
                    turret.setPower(0);
                    delay = new DelayStateChange(0.5, extraActions ? State.LIFT_UP2 : State.DONE);
                }
                if (Math.abs(turret.getCurrentPosition()) > MAX_SCAN_FROM_CENTER) {
                    // give up (fail fast™)
                    stateChange(State.IDLE);
                }
                break;
            case LIFT_UP1:
                if (liftController.isSatisfiedVertically()) {
                    stateChange(State.ROTATE1);
                }
                break;
            case LIFT_UP2:
                if (liftController.isSatisfiedVertically()) {
                    stateChange(State.EXTEND);
                }
                break;
            case EXTEND:
                if (liftController.isSatisfiedHorizontally()) {
                    delay = new DelayStateChange(0.5, State.CLAW_OPEN);
                }
                break;
            case CLAW_OPEN:
                delay = new DelayStateChange(0.5, State.RETRACT);
                break;
            case RETRACT:
                if (liftController.isSatisfiedHorizontally()) {
                    delay = new DelayStateChange(0.5, State.LIFT_DOWN);
                }
                break;
            case LIFT_DOWN:
                if (liftController.isSatisfiedVertically()) {
                    stateChange(State.DONE);
                }
                break;

        }
    }
}
