package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RunToClickCount.computeNextPower;
import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EncoderReader extends LinearOpMode {
//    CPR 8192, 35mm encoder wheels
    int targetEncCount = 7500;
    double lastUpdated = 0;
    boolean lastA = false;
    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            int pos = -frontRight.getCurrentPosition();
            int err = targetEncCount - pos;
            telemetry.addData("pos", pos);
            telemetry.addData("target", targetEncCount);
            double scaledPower = pos < targetEncCount ? Math.min(0.4, err/65536f)/2  : 0;
            frontRight.setPower(scaledPower);
            frontLeft.setPower(scaledPower);
            rearLeft.setPower(scaledPower);
            rearRight.setPower(scaledPower);
            if (runtime.seconds() - lastUpdated > 0.01) {
                double previousPower = frontRight.getPower();

                lastUpdated = runtime.seconds();
            }
            lastA = gamepad1.a;
            telemetry.update();
        }
    }
}
