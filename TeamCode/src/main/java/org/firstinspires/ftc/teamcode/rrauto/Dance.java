package org.firstinspires.ftc.teamcode.rrauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

@Config
@TeleOp(name = "RR: Dance", group = "$RR")
public class Dance extends LinearOpMode {
    public static double SPEEED = 20;
    public static double DIST_FIRST = 2;
    public static double DIST_SECOND = 4;
    public static double DIST_THIRD = 6;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        ArrayList<Trajectory> trags = new ArrayList<>();
        trags.add(drive.trajectoryBuilder(new Pose2d())
                .forward(DIST_FIRST * 12,
                        SampleMecanumDrive.getVelocityConstraint(SPEEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(DIST_SECOND, DIST_SECOND), 90,
                        SampleMecanumDrive.getVelocityConstraint(SPEEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(DIST_THIRD, DIST_THIRD), 180,
                        SampleMecanumDrive.getVelocityConstraint(SPEEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(0, 0), 0,
                        SampleMecanumDrive.getVelocityConstraint(SPEEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build());


        for (Trajectory t : trags) {
            drive.followTrajectory(t);
        }
    }
}
