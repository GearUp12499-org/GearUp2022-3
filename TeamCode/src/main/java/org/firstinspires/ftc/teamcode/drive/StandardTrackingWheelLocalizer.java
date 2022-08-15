package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.69; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.625; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 3.75; // in; offset of the lateral wheel

    // OLD TUNING DATA v1 | TUNING
    // OLD TUNING DATA v1 | BATCH 1 = 98.31229344142051
    // OLD TUNING DATA v1 | BATCH 2 = 98.73244343478059
    // OLD TUNING DATA v1 | BATCH 3 = 98.26362355716793
    // OLD TUNING DATA v1 | JJ SAYS = 98.46178077614168
    // OLD TUNING DATA v1 |
    // OLD TUNING DATA v1 | BATCH 1 = 99.17099944933457
    // OLD TUNING DATA v1 | BATCH 2 = 99.12105217287485
    // OLD TUNING DATA v1 |
    // OLD TUNING DATA v1 | BATCH 3 = 89.39152717340066 @90in
    // OLD TUNING DATA v1 | BATCH 4 = 89.55914563846005 @90in
    // OLD TUNING DATA v1 | BatcH 5 = 89.33550121378164

    // OLD TUNING DATA v1 | STRAFE
    // OLD TUNING DATA v1 | 37.5597354218156 @60
    // OLD TUNING DATA v1 | 49.63018773359754, 1.597455342 @60
//    public static double X_MULTIPLIER = 1.015887256; // Multiplier in the X direction
    public static double X_MULTIPLIER = 1; // Multiplier in the X direction
//    public static double Y_MULTIPLIER = 1.006388052; // Multiplier in the Y direction
    public static double Y_MULTIPLIER = 1; // Multiplier in the Y direction

    public Encoder leftEncoder, rightEncoder, frontEncoder;

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_left"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_right"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rear_right"));

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);  // if needed
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        //  If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method ~~ DONE July 23 2022 Miles ~~

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
        );
    }
}
