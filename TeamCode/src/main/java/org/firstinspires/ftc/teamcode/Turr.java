package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Turret liam you suck ", group = "!!!!!!!!!")
public class Turr extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        waitForStart();
        while (opModeIsActive()) turret.setPower(gamepad2.left_stick_x);
    }
}
