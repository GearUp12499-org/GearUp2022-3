package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.Map;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.Consumer;

public class DetectPoleV2 {
    public final DcMotor turret;
    public final DistanceSensor distanceSensor;
    public double lastDistance;

    public State getState() {
        return currentState;
    }

    public void beginScan(RotateDirection direction) {
        if (currentState != State.IDLE) return;
        this.rotateDirection = direction;
        stateChange(State.BEGIN);
    }

    public enum State {
        IDLE,
        WAITING,
        BEGIN,
        DONE, ROTATE1
    }

    public static final Map<State, Consumer<DetectPoleV2>> ON_ENTER_DEFAULTS = new HashMap<>();
    public static final Map<State, Consumer<DetectPoleV2>> ON_EXIT_DEFAULTS = new HashMap<>();

    public static final double SPEED = 0.5;

    static {
        ON_ENTER_DEFAULTS.put(State.IDLE, o -> {
            o.turret.setPower(0); // Force stop
        });
        ON_ENTER_DEFAULTS.put(State.DONE, o -> {
            o.turret.setPower(0); // Stop
        });
        ON_ENTER_DEFAULTS.put(State.BEGIN, o -> o.stateChange(State.ROTATE1));  // Change entrypoint
        ON_ENTER_DEFAULTS.put(State.ROTATE1, o -> {
            o.turret.setPower(SPEED * o.rotateDirection.powerModifier);
        });
    }

    private final Map<State, Consumer<DetectPoleV2>> onEnter = new HashMap<>(ON_ENTER_DEFAULTS);
    private final Map<State, Consumer<DetectPoleV2>> onExit = new HashMap<>(ON_EXIT_DEFAULTS);

    public DetectPoleV2(DcMotor turret, DistanceSensor distanceSensor) {
        this.turret = turret;
        this.distanceSensor = distanceSensor;
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

    private final class DelayStateChange {
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
    public double rotateSpeed = 0.5;

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
                // check if your mother is fat
                lastDistance = distanceSensor.getDistance(DistanceUnit.MM);

                if (lastDistance < 500) {
                    // stop
                    turret.setPower(0);
                    delay = new DelayStateChange(0.5, State.DONE);
                }
                if (Math.abs(turret.getCurrentPosition()) > 500) {
                    // stop
                    stateChange(State.IDLE);
                }
                break;
        }
    }
}
