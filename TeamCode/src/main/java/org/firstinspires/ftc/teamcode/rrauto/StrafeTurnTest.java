package org.firstinspires.ftc.teamcode.rrauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="RR: Strafe + Turn Test", group="$RR")
public class StrafeTurnTest extends LinearOpMode {
    public static double DIST = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        Trajectory t1 = drive.trajectoryBuilder(new Pose2d(0, 0, 0)).strafeLeft(48).build();
        drive.followTrajectory(t1);
        drive.turn(Math.toRadians(180));
        Trajectory t2 = drive.trajectoryBuilder(new Pose2d(t1.end().getX(), t1.end().getY(), Math.toRadians(180))).strafeLeft(48).build();
        drive.followTrajectory(t2);
//        drive.turn(180);
    }
}
