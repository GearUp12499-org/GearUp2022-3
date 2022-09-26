package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// 750 counts / 90deg
//
public class DetectPoleOneSensor {
    public static final class Result {
        public final boolean ok;
        public final int encoderValue;
        public final double estimatedDistance;
        private Result(boolean ok, int encoderValue, double estimatedDistance) {
            this.ok = ok;
            this.encoderValue = encoderValue;
            this.estimatedDistance = estimatedDistance;
        }

        public static Result success(int encoderValue, double estimatedDistance) {
            return new Result(true, encoderValue, estimatedDistance);
        }

        public static Result failure() {
            return new Result(false, 0, 0);
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

    public static double SCAN_SPEED = 1;
    public static double COMPENSATE_SPEED = 0.8;

    public static double COMP_WAIT_TIME = 0.5;
    private ElapsedTime timer = null;

    public static double EDGE_THRESHOLD = -1000;  // mm
    public static double GLOBAL_HOLD_THRESHOLD = 1000;
    public double holdThreshold = GLOBAL_HOLD_THRESHOLD;

    public double lastDist = 0;

    public static int SCAN_ENCODER_LIMIT = 375; // encoder counts; =45deg
    public static int ENCODER_NEARNESS = 50;

    public static boolean INITIAL_DIRECTION = true; // true => cw, false => ccw

    public State state = State.FAILED;

    private boolean currentDirection = INITIAL_DIRECTION;

    public DcMotor turret;
    public DistanceSensor sensor;

    public DetectPoleOneSensor(DcMotor turret, DistanceSensor sensor) {
        this.turret = turret;
        this.sensor = sensor;
    }
    public DetectPoleOneSensor(DcMotor turret, DistanceSensor sensor, double expectedDistanceAway) {
        this(turret, sensor);
        this.holdThreshold = expectedDistanceAway;
    }

    /**
     * Be within [x]deg of the pole
     * Go get the pole
     * Return the correct encoder count of the turret
     **/
    public void beginScan() {
        state = State.START;
    }

    private void halt() {
        turret.setPower(0);
    }

    public void update() {
        int turretEncoderCounts = turret.getCurrentPosition();
        double readDist = sensor.getDistance(DistanceUnit.MM);
        boolean hit = readDist - lastDist <= EDGE_THRESHOLD;
        boolean holdHit = readDist <= holdThreshold;

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
                    state = State.COMPENSATE_WAIT;
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
                } else if (hit) {
                    halt();
                    state = State.COMPENSATE_WAIT;
                }
                break;
            case COMPENSATE_WAIT:
                if (timer == null) {
                    timer = new ElapsedTime();
                }
                if (timer.seconds() > COMP_WAIT_TIME) {
                    state = State.COMPENSATE_TRANSITION;
                    timer = null;
                }
                break;
            case COMPENSATE_TRANSITION:
                if (holdHit) {
                    state = State.FINISH;
                    break;
                }
                currentDirection = !currentDirection;
                double sf = (currentDirection?1:-1)*COMPENSATE_SPEED;
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                turret.setPower(sf);
                state = State.COMPENSATE;
                break;
            case COMPENSATE:
                if (holdHit) {
                    halt();
                    state = State.FINISH;
                }
                break;
            default:
                halt();
        }

        lastDist = readDist;
    }
}
