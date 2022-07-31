package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Display Encoders...", group="Tasks")
public class DisplayEncoders extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        waitForStart();
        final double C = 745.027025034;
        while (opModeIsActive()) {
            telemetry.addData("encoderLeft [1]", encoderLeft.getCurrentPosition());
            telemetry.addData("encoderLeft [1] distance", encoderLeft.getCurrentPosition()/C);
            telemetry.addData("encoderRight [0]", encoderRight.getCurrentPosition());
            telemetry.addData("encoderRight [0] distance", encoderRight.getCurrentPosition()/C);
            telemetry.addData("encoderRear [2]", encoderRear.getCurrentPosition());
            telemetry.addData("encoderRear [2] distance", encoderRear.getCurrentPosition()/C);
            telemetry.update();
        }
    }
}
