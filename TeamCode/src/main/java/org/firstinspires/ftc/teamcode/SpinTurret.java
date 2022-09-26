package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="SpinTurret", group="!!!!!!!!")
@Config
public class SpinTurret extends LinearOpMode {
    private static double AMOUNT = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        SharedHardware.prepareHardware(hardwareMap);
        waitForStart();
        SharedHardware.turret.setPower(AMOUNT);
        while (opModeIsActive()) {
            SharedHardware.turret.setPower(gamepad2.left_stick_x);
            telemetry.addData("encoder", SharedHardware.turret.getCurrentPosition());
            telemetry.update();
        }
        SharedHardware.turret.setPower(0);
    }
}
