package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.IOControl;

@Autonomous(name="Speedtest", group="!test")
public class Speedtest extends LinearOpMode {
    public double testTime = 5000; // ms
    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry merged = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        IOControl ioControl = new IOControl(hardwareMap);  // connect to the hardware
        merged.addLine("Speedtest");
        merged.addLine();
        merged.addLine("This will test the poll rate of the distance sensor.");
        merged.addLine("Please don't press STOP while the test is running.");
        merged.update();
        ElapsedTime e = new ElapsedTime();
        waitForStart();
        merged.addLine("Running: Poll rate test for " + testTime + "ms");
        merged.update();
        e.reset();
        int count = 0;
        while (opModeIsActive() && e.milliseconds() < testTime) {
            count++;
            ioControl.distSensor.getDistance(DistanceUnit.MM);
        }
        double duration = e.milliseconds();
        merged.addLine("Counts: " + count + " total");
        merged.addLine("Poll rate: " + count / (duration / 1000) + " per second");
        merged.addLine("(It is safe to STOP the opmode now.)");
        merged.update();

        //noinspection StatementWithEmptyBody
        while (opModeIsActive()) ;
    }
}
