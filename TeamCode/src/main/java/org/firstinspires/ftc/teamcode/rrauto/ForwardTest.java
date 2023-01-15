package org.firstinspires.ftc.teamcode.rrauto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

@Config
@TeleOp(name = "RR: Forward Test", group = "$RR")
public class ForwardTest extends LinearOpMode {
    public static double DIST = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        telemetry = FtcDashboard.getInstance().getTelemetry();
        waitForStart();
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                .forward(DIST)
                .build());
        Localizer localizer = drive.getLocalizer();
        if (localizer instanceof StandardTrackingWheelLocalizer) {
            StandardTrackingWheelLocalizer stwl = (StandardTrackingWheelLocalizer) localizer;
            telemetry.addData("left", stwl.leftEncoder.getCurrentPosition());
            telemetry.addData("right", stwl.rightEncoder.getCurrentPosition());
            telemetry.addData("front", stwl.frontEncoder.getCurrentPosition());
        } else {
            telemetry.addData("bad localizer", "oh no");
        }
        telemetry.update();
    }
}
