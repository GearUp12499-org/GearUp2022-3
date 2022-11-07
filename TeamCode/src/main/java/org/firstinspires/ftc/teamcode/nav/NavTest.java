package org.firstinspires.ftc.teamcode.nav;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import java.util.Locale;

public class NavTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        EncoderNavigation nav = new EncoderNavigation(
                frontLeft, frontRight, rearLeft, rearRight,
                encoderLeft, encoderRight, encoderRear
        );
        waitForStart();
        nav.moveForward(5);
        while (opModeIsActive()) {
            nav.asyncLoop();
            telemetry.addData("left", nav.leftOdom.getCurrentPosition());
            telemetry.addData("right", nav.rightOdom.getCurrentPosition());
            telemetry.addData("center", nav.frontOdom.getCurrentPosition());
            telemetry.addLine();
            telemetry.addLine(String.format(Locale.US, "Completion: %.2f", nav.lastKnownProgress / (double) nav.targetEncoderValue));
            telemetry.update();
        }
    }
}
