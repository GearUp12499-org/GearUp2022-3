package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "When the debugging", group = "!!!!!!!!!")
@Disabled
public class Turr extends LinearOpMode {

    public static final double SEC_PER_IN = (double) 1 / 25;
    private static final double SEC_PER_IN_STR = (double) 1 / 18;

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

    public void doTheStrafeRightThing(double timing) {
//        for  bak
//        bak  for
        double time = Math.abs(timing);
        int sig = (int) Math.signum(timing);
        frontLeft.setPower(0.5 * sig);
        rearLeft.setPower(-0.5 * sig);
        rearRight.setPower(0.5 * sig);
        frontRight.setPower(-0.5 * sig);
        sleep((long) (timing * 1000));
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontRight.setPower(0);
    }

    public void zone1() {
        doTheMoveForwardThing(2);
        doTheStrafeRightThing(-1.75);
        doTheMoveForwardThing(34);
    }

    public void zone2() {
        doTheMoveForwardThing(36);
    }

    public void zone3() {
        doTheMoveForwardThing(2);
        doTheStrafeRightThing(1.75);
        doTheMoveForwardThing(34);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        waitForStart();
    }
}
