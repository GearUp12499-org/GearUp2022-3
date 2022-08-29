package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="MecanumDrive_v1")
public class SimpleMecanum extends LinearOpMode {

    public void onAPressed() {
//        telemetry.speak("You pressed the ayy button");
        gamepad1.rumble(1, 1, 500);
    }

    public static double SPEED_LIMIT = 0.75;
    private boolean la = false;
    private void processA() {
        if (gamepad1.a && !la) onAPressed();
        la = gamepad1.a;
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prepareHardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            processA();

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

            frontLeft.setPower(frontLeftPower * SPEED_LIMIT);
            rearLeft.setPower(rearLeftPower * SPEED_LIMIT);
            frontRight.setPower(frontRightPower * SPEED_LIMIT);
            rearRight.setPower(rearRightPower * SPEED_LIMIT);

            int pos = frontRight.getCurrentPosition();
            telemetry.addData("pos", pos);

            telemetry.update();
        }
    }
}
