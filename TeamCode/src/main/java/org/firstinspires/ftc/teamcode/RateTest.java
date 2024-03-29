package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@Autonomous(name = "OdometryRateTest", group = "Test")
@Disabled
public class RateTest extends LinearOpMode {
    public static final int CLICK_TARGET = 50000;
    public static final double SPEED = 0.2;
    private boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        telemetry.addLine("CONFIGURATION:");
        telemetry.addData("Clicks", CLICK_TARGET);
        telemetry.addData("Speed", SPEED);
        telemetry.update();
        waitForStart();
        frontLeft.setPower(SPEED);
        frontRight.setPower(SPEED);
        rearLeft.setPower(SPEED);
        rearRight.setPower(SPEED);
        ElapsedTime timer = new ElapsedTime();
        // Start a timer, and keep track of the number of encoder ticks
        int pos = 0;
        while (opModeIsActive()) {
            pos = -frontRight.getCurrentPosition();
            telemetry.addData("clicks:", pos);
            if (pos > CLICK_TARGET) {
                break;
            }
            telemetry.addData("time:", timer.milliseconds() / 1000.0);
            telemetry.update();
        }
        double finishTime = timer.milliseconds() / 1000.0;
        telemetry.addData("Time:", finishTime);
        // Stop the motors
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        // Calculate the rate of encoder ticks per second
        double rate = pos / finishTime;
        telemetry.addLine("Rate per second (slow):");
        telemetry.addLine(String.format(Locale.US, "%1.2f", rate));
        int realRate = (int) (rate * (1 / SPEED));
        telemetry.addLine("PUT THIS IN THE PROGRAM:");
        telemetry.addLine(String.format(Locale.US, "%d", realRate));
        telemetry.addLine();
        telemetry.addLine("Press and hold A to stop");
        telemetry.update();

        double startA = Double.MAX_VALUE;
        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) {
                startA = timer.milliseconds();
            }
            if (gamepad1.a && timer.milliseconds() - startA > 1000) {
                break;
            }
            lastA = gamepad1.a;
        }
    }
}