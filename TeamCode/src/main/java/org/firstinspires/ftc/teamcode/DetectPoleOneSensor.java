package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

// 750 counts / 90deg
//
public class DetectPoleOneSensor {
    public static final class Result {
        public enum State {
            SUCCESS("Success"),
            FAILURE("Failure"),
            NO_RESULT_YET("Running");
            final String name;
            State(String name) {
                this.name = name;
            }

            @NonNull
            @Override
            public String toString() {
                return name;
            }
        }
        public final State state;
        public final int encoderValue;
        public final double estimatedDistance;
        private Result(State state, int encoderValue, double estimatedDistance) {
            this.state = state;
            this.encoderValue = encoderValue;
            this.estimatedDistance = estimatedDistance;
        }

        public static Result success(int encoderValue, double estimatedDistance) {
            return new Result(State.SUCCESS, encoderValue, estimatedDistance);
        }

        public static Result failure() {
            return new Result(State.FAILURE, 0, 0);
        }

        public static Result na() {
            return new Result(State.NO_RESULT_YET, 0, 0);
        }

        @NonNull
        @Override
        public String toString() {
            if (state != State.SUCCESS) return String.format(Locale.ENGLISH, "[%s]", state.name());
            else return String.format(Locale.ENGLISH, "[%s] %dc %fmm", state.name(), encoderValue, estimatedDistance);
        }
    }

    public enum State {
        START,
        SCAN_PASS1,
        SCAN_TRANSITION,
        SCAN_PASS2, // twice distance, opp direction
        COMPENSATE_WAIT,
        COMPENSATE_TRANSITION,
        COMPENSATE,
        FINISH,
        FAILED
    }

    public static double SCAN_SPEED = 0.5;
    public static double COMPENSATE_SPEED = 0.3;

    public static double COMP_WAIT_TIME = 0.5;
    private ElapsedTime timer = null;

    public static double EDGE_THRESHOLD = -1000;  // mm
    public double expectedDistance;

    public double lastDist = 0;

    public static int SCAN_ENCODER_LIMIT = 375; // encoder counts; =45deg
    public static int ENCODER_NEARNESS = 50;
    public static int COMP_ENCODER_NEARNESS = 25;

    public static boolean INITIAL_DIRECTION = true; // true => cw, false => ccw

    public State state = State.FAILED;

    private boolean currentDirection = INITIAL_DIRECTION;

    private Result result;

    public DcMotor turret;
    public DistanceSensor sensor;

    public DetectPoleOneSensor(DcMotor turret, DistanceSensor sensor, double expectedDistance) {
        this.turret = turret;
        this.sensor = sensor;
        this.expectedDistance = expectedDistance;
    }

    /**
     * Be within [x]deg of the pole
     * Go get the pole
     * Return the correct encoder count of the turret
     **/
    public void beginScan() {
        state = State.START;
        result = Result.na();
    }

    private void halt() {
        turret.setPower(0);
    }

    public void update() {
        int turretEncoderCounts = turret.getCurrentPosition();
        double readDist = sensor.getDistance(DistanceUnit.MM);
        boolean hit = readDist - lastDist <= EDGE_THRESHOLD && readDist <= expectedDistance;

        // Garbage state machine that Dad won't be proud of
        switch (state) {
            case START:
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                currentDirection = INITIAL_DIRECTION;

                turret.setTargetPosition(SCAN_ENCODER_LIMIT);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(SCAN_SPEED);
                state = State.SCAN_PASS1;
                break;
            case SCAN_PASS1:
                if (Math.abs(turretEncoderCounts - turret.getTargetPosition()) < ENCODER_NEARNESS) {
                    state = State.SCAN_TRANSITION;
                } else if (hit) {
                    halt();
                    state = State.COMPENSATE_TRANSITION;
                }
                break;
            case SCAN_TRANSITION:
                currentDirection = !currentDirection;

                turret.setTargetPosition(SCAN_ENCODER_LIMIT * -1);
                state = State.SCAN_PASS2;
                break;
            case SCAN_PASS2:
                if (Math.abs(turretEncoderCounts - turret.getTargetPosition()) < ENCODER_NEARNESS) {
                    state = State.FAILED;
                    result = Result.failure();
                } else if (hit) {
                    halt();
                    state = State.COMPENSATE_TRANSITION;
                }
                break;
            case COMPENSATE_TRANSITION:
                turret.setTargetPosition(turret.getCurrentPosition());
                turret.setPower(COMPENSATE_SPEED);
                state = State.COMPENSATE_WAIT;
                break;
            case COMPENSATE_WAIT:
                if (timer == null) {
                    timer = new ElapsedTime();
                }
                if (timer.seconds() > COMP_WAIT_TIME) {
                    state = State.COMPENSATE;
                    timer = null;
                }
                break;
            case COMPENSATE:
                if (Math.abs(turretEncoderCounts - turret.getTargetPosition()) < COMP_ENCODER_NEARNESS) {
                    halt();
                    state = State.FINISH;
                    result = Result.success(turretEncoderCounts, readDist);
                }
                break;
            default:
                halt();
        }

        lastDist = readDist;
    }

    public Result getResult() {
        return result;
    }

    public Result holdForResultSync() {
        while (result.state == Result.State.NO_RESULT_YET) {
            update();
        }
        return getResult();
    }
}
