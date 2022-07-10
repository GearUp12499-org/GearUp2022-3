package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EncoderReader extends LinearOpMode {
    int targetEncCount = 0;
    boolean lastA = false;
    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            int pos = frontRight.getCurrentPosition();
            telemetry.addData("pos", pos);
            if (!lastA && gamepad1.a) {
                targetEncCount += 1000;
            }
            double scaledPower = Math.min(1, (targetEncCount - pos) / 1000);
            frontRight.setPower(scaledPower);
            frontLeft.setPower(scaledPower);
            rearLeft.setPower(scaledPower);
            rearRight.setPower(scaledPower);
            lastA = gamepad1.a;
            telemetry.update();
        }
    }
}
