package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "vroom vroom", group = "!!!!!!!!")
public class Vroom extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) lift.up();
            else if (gamepad1.b) lift.down();
            else lift.stop();
            if (gamepad1.x) servo.setPosition(0);
            else if (gamepad1.y) servo.setPosition(0.4);
            telemetry.addData("avg encoder", lift.getEncoderCounts());
            telemetry.update();
            lift.safety();
        }
    }
}
