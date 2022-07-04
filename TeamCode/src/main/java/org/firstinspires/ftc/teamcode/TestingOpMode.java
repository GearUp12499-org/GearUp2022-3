package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Date;

@TeleOp(name="Testing OpMode")
public class TestingOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        long timeStart = new Date().getTime();
        while (!(isStopRequested() || gamepad1.a)) {
            telemetry.addLine("It works, yay!!");
            telemetry.addLine();
            telemetry.addLine("Press 'a <1>' to stop");
            telemetry.addLine();
            telemetry.addData("Runtime (ms)", "%d", new Date().getTime() - timeStart);
            telemetry.update();
        }
    }
}
