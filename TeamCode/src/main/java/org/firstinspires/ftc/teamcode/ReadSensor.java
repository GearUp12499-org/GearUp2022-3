package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Read sensor", group="!!!!!!!!")
public class ReadSensor extends LinearOpMode {
    double lastDistance = 0;
    public static final double LIMIT = 1000;  // mm
//    public static final double POP_AFTER = 7000;

    @Override
    public void runOpMode() throws InterruptedException {
        IOControl ioControl = new IOControl(hardwareMap);  // connect to the hardware
        Telemetry mergedTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();
        while (opModeIsActive()) {
            double currentDistance = ioControl.distSensor.getDistance(DistanceUnit.MM);
//            if (now >= POP_AFTER) now = POP_AFTER;
            double d = currentDistance - lastDistance;
            if (Math.abs(d) >= LIMIT && d < 0) {
                mergedTelemetry.addLine("Hit delta " + d);
                mergedTelemetry.addLine("Distance " + currentDistance + "mm"); // read the distance
                mergedTelemetry.update();
                telemetry.speak("hit");
            }
            lastDistance = currentDistance;
        }
    }
}
