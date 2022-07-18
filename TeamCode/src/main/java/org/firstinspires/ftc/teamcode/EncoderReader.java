package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EncoderReader extends LinearOpMode {
//    CPR 8192, 35mm encoder wheels
    double lastUpdated = 0;
    boolean lastA = false;
    double finalTime = 0;
    double POWER = .9;
    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        waitForStart();
        runtime.reset();
        double target = MovementLibrary.getTargetEncoder(0.5, POWER);
        while (opModeIsActive()) {
            int pos = frontRight.getCurrentPosition();
            telemetry.addData("pos", pos);
            telemetry.addData("target", target);
            telemetry.addData("dt", runtime.seconds() - lastUpdated);
            if (pos < target) {
                frontRight.setPower(POWER);
                frontLeft.setPower(POWER);
                rearRight.setPower(POWER);
                rearLeft.setPower(POWER);
            } else {
                frontRight.setPower(0.0);
                frontLeft.setPower(0.0);
                rearRight.setPower(0.0);
                rearLeft.setPower(0.0);
                finalTime = runtime.seconds();
                break;
            }
            lastA = gamepad1.a;
            telemetry.update();
        }
        while (opModeIsActive()) {
            int pos = frontRight.getCurrentPosition();
            telemetry.addData("pos", pos);
            telemetry.addData("time", finalTime);
            telemetry.update();
        }
    }
}
