package org.firstinspires.ftc.teamcode.rrauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@TeleOp(name="RR: Forward Test", group="$RR")
public class ForwardTest extends LinearOpMode {
    public static double DIST = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        drive.followTrajectory(drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                        .forward(DIST)
                .build());
    }
}
