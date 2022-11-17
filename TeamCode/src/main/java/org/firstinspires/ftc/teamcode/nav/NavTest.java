package org.firstinspires.ftc.teamcode.nav;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import java.util.Locale;

@Autonomous(name="NavTest", group="!!!!!!!!")
public class NavTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        EncoderNavigation nav = new EncoderNavigation(
                frontLeft, frontRight, rearLeft, rearRight,
                encoderLeft, encoderRight, encoderRear
        );
        waitForStart();
        nav.moveForward(12);
        nav.strafeLeft(12);
        nav.moveForward(-12);
        nav.strafeRight(12);
        while (opModeIsActive()) {
            nav.asyncLoop();
            nav.dumpTelemetry(telemetry);
            telemetry.update();
        }
    }
}
