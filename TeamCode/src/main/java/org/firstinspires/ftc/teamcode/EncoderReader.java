package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.RunToClickCount.computeNextPower;
import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EncoderReader extends LinearOpMode {
//    CPR 8192, 35mm encoder wheels
    int targetEncCount = 50000;
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
            telemetry.addData("dt", runtime.seconds() - lastUpdated);
            double previousPower = frontRight.getPower();
            telemetry.addData("power", previousPower);
            if (runtime.seconds() - lastUpdated > 0.01) {
                double scaledPower = computeNextPower(err, previousPower, runtime.seconds() - lastUpdated);
                frontRight.setPower(scaledPower);
                frontLeft.setPower(scaledPower);
                rearLeft.setPower(scaledPower);
                rearRight.setPower(scaledPower);
                lastUpdated = runtime.seconds();
            }
            lastA = gamepad1.a;
            telemetry.update();
        }
    }
}
