package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.nav.EncoderNavigation;
import org.firstinspires.ftc.teamcode.nav.Paths;

@TeleOp(name="When the debugging", group = "!!!!!!!!!")
public class Turr extends LinearOpMode {

    public static final double SEC_PER_IN = (double)1/25;
    private static final double SEC_PER_IN_STR = (double)1/18;

    public void doTheMoveForwardThing(double inc) {  // i hate this so much
        double time = Math.abs(inc);
        int sig = (int) Math.signum(inc);
        frontLeft.setPower(0.5 * sig);
        rearLeft.setPower(0.5 * sig);
        rearRight.setPower(0.5 * sig);
        frontRight.setPower(0.5 * sig);
        sleep((long) (SEC_PER_IN * time * 1000));
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontRight.setPower(0);
    }

    public void doTheStrafeRightThing(double fuckOff) {
//        for  bak
//        bak  for
        double time = Math.abs(fuckOff);
        int sig = (int) Math.signum(fuckOff);
        frontLeft.setPower(0.5 * sig);
        rearLeft.setPower(-0.5 * sig);
        rearRight.setPower(0.5 * sig);
        frontRight.setPower(-0.5 * sig);
        sleep((long) (fuckOff * 1000));
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontRight.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        waitForStart();
//        EncoderNavigation nav = new EncoderNavigation(frontLeft, frontRight, rearLeft, rearRight, encoderLeft, encoderRight, encoderRear);
//        Paths paths = new Paths(nav);
//        nav.moveForward(2);
//        paths.zone2();
//        while (opModeIsActive()) {
//            nav.asyncLoop();
//            nav.dumpTelemetry(telemetry);
//            telemetry.update();
//        }
//        doTheMoveForwardThing(24*1.5);
        doTheStrafeRightThing(1.75);
    }
}
