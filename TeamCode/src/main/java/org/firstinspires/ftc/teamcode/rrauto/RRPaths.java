package org.firstinspires.ftc.teamcode.rrauto;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class RRPaths {
    public static final double TILE = 24;
    public static final boolean USE_STRAFE = false;

    public static TrajectorySequence zone1(SampleMecanumDrive drive) {
        if (USE_STRAFE) {
            return drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(1)
                    .strafeLeft(TILE)
                    .forward(1.5 * TILE)
                    .build();
        }
        return drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1)
                .turn(toRadians(-90))
                .forward(TILE)
                .turn(toRadians(90))
                .forward(1.5 * TILE)
                .build();
    }
    public static TrajectorySequence zone2(SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1)
                .forward(1.5 * TILE)
                .build();
    }
    public static TrajectorySequence zone3(SampleMecanumDrive drive) {
        if (USE_STRAFE) {
            return drive.trajectorySequenceBuilder(new Pose2d())
                    .forward(1)
                    .strafeRight(TILE)
                    .forward(1.5 * TILE)
                    .build();
        }
        return drive.trajectorySequenceBuilder(new Pose2d())
                .forward(1)
                .turn(toRadians(90))
                .forward(TILE)
                .turn(toRadians(-90))
                .forward(1.5 * TILE)
                .build();
    }
}
