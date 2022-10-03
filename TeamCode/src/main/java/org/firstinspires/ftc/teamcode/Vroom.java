package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "vroom vroom", group = "!!!!!!!!")
public class Vroom extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor l1 = hardwareMap.get(DcMotor.class, "lift1");
        l1.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor l2 = hardwareMap.get(DcMotor.class, "lift2");
        l2.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            double power = gamepad1.a ? 1 : 0;
            l1.setPower(power);
            l2.setPower(power);
        }
    }
}
