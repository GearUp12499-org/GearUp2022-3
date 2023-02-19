package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;

@TeleOp(name = "<wip> Tele2Op", group = "!!!!!!!!")
public class Tele2Op extends LinearCleanupOpMode {
    @Override
    public void main() throws InterruptedException {
        prepareHardware(hardwareMap);
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        telemetry.addLine("Initializing IMU...");
        telemetry.update();
        imu.initialize(parameters);
        telemetry.addLine("Waiting...");
        telemetry.update();
        while (opModeInInit()) stopMaybe();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            stopMaybe();

            if (gamepad1.start) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y; // reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("he", heading);
            telemetry.update();
            double rotX = Math.cos(-heading) * x - Math.sin(-heading) * y;
            double rotY = Math.sin(-heading) * x + Math.cos(-heading) * y;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPow = (rotY + rotX + rx) / denominator;
            double backLeftPow = (rotY - rotX + rx) / denominator;
            double frontRightPow = (rotY - rotX - rx) / denominator;
            double rearRightPow = (rotY + rotX - rx) / denominator;

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
