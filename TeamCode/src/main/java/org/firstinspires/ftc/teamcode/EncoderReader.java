package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class EncoderReader extends LinearOpMode {
    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            int pos = frontRight.getCurrentPosition();
            telemetry.addData("pos", pos);
            telemetry.update();
        }
    }
}
