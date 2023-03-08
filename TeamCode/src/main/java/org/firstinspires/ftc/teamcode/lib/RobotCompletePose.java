package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Lift;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

public class RobotCompletePose {
    /*
     * 1. Go to known safe position above the chassis
     * 2. Linear path to goal
     */

    /**
     * The robot has to travel to a safe point before it is allowed to
     * cross the zone threshold.
     * Maintainers: ensure the 2nd args are all the same, and match the ZONE_THRESHOLD
     */
    public static final RobotCompletePose[] SAFE_PT = {new RobotCompletePose(0, 300, 0),};

    /**
     * How much is the turret allowed to turn in the lower zone?
     * TODO put actual value here
     * 0 = not at all
     */
    public static final int HORIZONTAL_CLEARANCE = 0;

    /**
     * How close to a safe point is close enough?
     */
    public static final int SAFE_PT_CLEAR_DISTANCE = 0;

    /**
     * Defines where the upper and lower zones are. Upper zone has free movement,
     * while the lower zone has very restricted movement to avoid colliding with the
     * chassis.
     */
    public static final int ZONE_THRESHOLD = 300;

    // Robot capture information
    public final int horizontalLift;
    public final int verticalLift;
    public final int turretPos;

    public RobotCompletePose(int horizontalLift, int verticalLift, int turret) {
        this.horizontalLift = horizontalLift;
        this.verticalLift = verticalLift;
        this.turretPos = turret;
    }

    /**
     * Capture the position of the robot right now based
     * on the current positions of the lift and turret motors.
     *
     * @param l lift control object
     * @return capture of the robot's current position
     */
    public static RobotCompletePose captureImmediatePosition(Lift l) {
        return captureImmediatePosition(l, turret);
    }

    private static RobotCompletePose captureImmediatePosition(Lift l, DcMotor turret) {
        return new RobotCompletePose(l.liftHorizontal.getCurrentPosition(), l.getFakedCurrentVerticalCounts(), turret.getCurrentPosition());
    }

    /**
     * Capture the position of the robot at the end of the current motion based
     * on the target positions of the lift and turret motors.
     *
     * @param l lift control object
     * @return capture of the robot's future position
     */
    public static RobotCompletePose captureFuturePosition(Lift l) {
        return captureFuturePosition(l, turret);
    }

    private static RobotCompletePose captureFuturePosition(Lift l, DcMotor turret) {
        // If the lift is in RUN_TO_POSITION mode, use the target position.
        // Otherwise, use the current position.
        int turretP = turret.getMode() == DcMotor.RunMode.RUN_TO_POSITION ? turret.getTargetPosition() : turret.getCurrentPosition();
        int lHP = l.liftHorizontal.getMode() == DcMotor.RunMode.RUN_TO_POSITION ? l.liftHorizontal.getTargetPosition() : l.liftHorizontal.getCurrentPosition();
        return new RobotCompletePose(lHP, l.getFakedVerticalCount(), turretP);
    }

    /**
     * Function to compute the "cost" of moving from one position to another.
     * This is used to determine the best path to take.
     *
     * @param other the other position
     * @return weighted cost of moving from this position to the other
     */
    public double cost(RobotCompletePose other) {
        final double HLIFT_W = 1;
        final double VLIFT_W = 1;
        final double TURRET_W = 1;
        return HLIFT_W * Math.abs(this.horizontalLift - other.horizontalLift) + VLIFT_W * Math.abs(this.verticalLift - other.verticalLift) + TURRET_W * Math.abs(this.turretPos - other.turretPos);
    }

    /**
     * Actual distance between two positions.
     */
    public double distance(RobotCompletePose other) {
        return Math.sqrt(Math.pow(this.horizontalLift - other.horizontalLift, 2) + Math.pow(this.verticalLift - other.verticalLift, 2) + Math.pow(this.turretPos - other.turretPos, 2));
    }

    /**
     * Find the nearest position in a list of positions.
     *
     * @param others list of positions
     * @return the nearest position
     */
    public RobotCompletePose nearest(RobotCompletePose... others) {
        // collect to a list
        List<RobotCompletePose> othersList = new ArrayList<>(Arrays.asList(others));
        return nearest(othersList);
    }

    /**
     * Find the nearest position in a list of positions.
     *
     * @param others list of positions
     * @return the nearest position
     */
    public RobotCompletePose nearest(List<RobotCompletePose> others) {
        RobotCompletePose best = null;
        double bestCost = Double.MAX_VALUE;
        for (RobotCompletePose other : others) {
            if (this.cost(other) < bestCost) {
                best = other;
                bestCost = this.cost(other);
            }
        }
        return best;
    }

    /**
     * Determine if the robot needs to switch between the upper and lower zones
     * to travel to this one.
     * If true, the robot needs to go to a safe point before it can continue.
     *
     * @param current current position of the robot
     * @return switch required?
     */
    public boolean needSwitch(RobotCompletePose current) {
        return ((current.verticalLift < ZONE_THRESHOLD && this.verticalLift >= ZONE_THRESHOLD) || (current.verticalLift >= ZONE_THRESHOLD && this.verticalLift < ZONE_THRESHOLD)) && distance(nearest(SAFE_PT)) > SAFE_PT_CLEAR_DISTANCE;
    }

    private void turretTo(int pos) {
        final int TURRET_ALLOWABLE_ERROR = 50;
        final int SLOWER = 300;
        int a = Math.abs(turret.getCurrentPosition() - pos);
        if (a > TURRET_ALLOWABLE_ERROR) {
            if (turret.getCurrentPosition() < pos) {
                if (a < SLOWER)
                    turret.setPower(0.2);
                else
                    turret.setPower(0.6);
            } else {
                if (a < SLOWER)
                    turret.setPower(-0.2);
                else
                    turret.setPower(-0.6);
            }
        }
    }

    public void runAsync(Lift l, @Nullable Telemetry writeTo) {
        // SharedHardware or bust
        if (writeTo != null) {
            writeTo.addLine("Pose runner:");
        }
        RobotCompletePose current = captureImmediatePosition(l);
        RobotCompletePose target = needSwitch(current) ? nearest(SAFE_PT) : this;
        if (writeTo != null) {
            writeTo.addLine("  going to " + target);
        }
        l.setVerticalTargetManual(target.verticalLift);
        l.setHorizontalTargetManual(target.horizontalLift);
        l.update();

        int deltaTurret = Math.abs(target.turretPos - this.turretPos); // 0 if not going to safe point
        if (deltaTurret > HORIZONTAL_CLEARANCE) {
            turret.setPower(0);
            return;
        }
        if (turret.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        turretTo(target.turretPos);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(Locale.getDefault(), "RobotCompletePose{hl=%d, vl=%d, t=%d}", horizontalLift, verticalLift, turretPos);
    }

    @Override
    public boolean equals(@Nullable Object obj) {
        if (!(obj instanceof RobotCompletePose)) {
            return false;
        }
        RobotCompletePose other = (RobotCompletePose) obj;
        return this.horizontalLift == other.horizontalLift && this.verticalLift == other.verticalLift && this.turretPos == other.turretPos;
    }
}
