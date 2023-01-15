package org.firstinspires.ftc.teamcode.rrauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@TeleOp(name = "RR: Strafe + Turn Test", group = "$RR")
public class StrafeTurnTest extends LinearOpMode {
    public static double DIST = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        waitForStart();
        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .strafeLeft(48).turn(Math.toRadians(180))
                .strafeLeft(48).turn(Math.toRadians(180))
                .build();
        drive.followTrajectorySequence(t1);
//        drive.turn(180);
    }
}
