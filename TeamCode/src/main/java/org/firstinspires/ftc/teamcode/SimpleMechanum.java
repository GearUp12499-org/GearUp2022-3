package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing")
public class SimpleMechanum extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private void prepareHardware() {
        // Correct names as of July 9 2022
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left");
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight = hardwareMap.get(DcMotor.class, "rear_right");
    }

    @Override
    public void runOpMode() {
        prepareHardware();
        waitForStart();
        while (opModeIsActive()) {
            // Literally stolen from gm0
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
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
