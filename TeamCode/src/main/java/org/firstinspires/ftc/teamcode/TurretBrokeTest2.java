package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TurretBrokeTest2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
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
