package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TurretBroke extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor turret = hardwareMap.get(DcMotor.class, "turret");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a) {
                turret.setPower(0.5);
            } else if (gamepad2.b) {
                turret.setPower(-0.5);
            } else {
                turret.setPower(0);
            }

            telemetry.addData("turret power", turret.getPower());
            telemetry.update();
        }
        turret.setPower(0);
    }
}
