package org.firstinspires.ftc.teamcode.nav;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "NavTest", group = "!!!!!!!!")
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
