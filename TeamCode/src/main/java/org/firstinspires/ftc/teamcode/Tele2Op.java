package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;

@TeleOp(name = "<wip> Tele2Op", group = "!!!!!!!!")
public class Tele2Op extends LinearCleanupOpMode {
    @Override
    public void main() throws InterruptedException {
        prepareHardware(hardwareMap);
        while (opModeInInit()) stopMaybe();
        waitForStart();
        while (opModeIsActive()) {
            stopMaybe();

            double y = -gamepad1.left_stick_y; // reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPow = (y + x + rx) / denominator;
            double backLeftPow = (y - x + rx) / denominator;
            double frontRightPow = (y - x - rx) / denominator;
            double rearRightPow = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPow);
            rearLeft.setPower(backLeftPow);
            frontRight.setPower(frontRightPow);
            rearRight.setPower(rearRightPow);
        }
    }

    /**
     * NEVER sleep or wait in this method. Only use this to stop motors,
     * clean up sensors & cameras, and generally stop the robot from moving.
     */
    @Override
    public void cleanup() {
        if (frontLeft != null) {
            frontLeft.setPower(0);
        }
        if (rearLeft != null) {
            rearLeft.setPower(0);
        }
        if (frontRight != null) {
            frontRight.setPower(0);
        }
        if (rearRight != null) {
            rearRight.setPower(0);
        }
    }
}
