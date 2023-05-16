package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Kids Drive Robots 1")
public class KidsDriveRobots1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            // Literally stolen from gm0
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1) * 2;
            double frontLeftPower = (y + x + rx) / denominator;
            double rearLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double rearRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
        }
    }
}
