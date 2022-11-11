package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="stupid teleop", group="!!!!!!!!")
public class VroomToEncoder extends LinearOpMode {
    boolean lastUp = false;
    boolean lastDown = false;
    int c = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            boolean up = (-gamepad1.left_stick_y) > 0.5;
            boolean down = (-gamepad1.left_stick_y) < -0.5;
            telemetry.addData("stick", -gamepad1.left_stick_y);
            telemetry.addData("current encoder", lift.liftVertical1.getCurrentPosition());
            telemetry.addData("target  encoder", lift.liftVertical1.getTargetPosition());
            telemetry.addData("up counter", c);
            telemetry.update();
            lift.update();
            if (up && !lastUp) {
                lift.goUp();
                c++;
            }
            if (down && !lastDown) {
                lift.goDown();
            }
            if (gamepad1.x) servo.setPosition(0);
            else if (gamepad1.y) servo.setPosition(0.4);
            lastUp = up;
            lastDown = down;
        }
    }
}
