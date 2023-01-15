package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

/*
 * This is a simple routine to test turning capabilities.
 */
@Config
@Autonomous(group = "drive")
public class TurnTest extends LinearOpMode {
    public static double ANGLE = 180; // deg

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        telemetry = FtcDashboard.getInstance().getTelemetry();

        waitForStart();

        if (isStopRequested()) return;

        drive.turn(Math.toRadians(ANGLE));

        Localizer localizer = drive.getLocalizer();
        if (localizer instanceof StandardTrackingWheelLocalizer) {
            StandardTrackingWheelLocalizer stwl = (StandardTrackingWheelLocalizer) localizer;
            telemetry.addData("left", stwl.leftEncoder.getCurrentPosition());
            telemetry.addData("right", stwl.rightEncoder.getCurrentPosition());
            telemetry.addData("front", stwl.frontEncoder.getCurrentPosition());
        }
        telemetry.update();
    }
}
