package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="stupid teleop", group="!!!!!!!!")
public class VroomToEncoder extends LinearOpMode {
    boolean lastUp = false;
    boolean lastDown = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        while (opModeIsActive()) {
            boolean up = (-gamepad1.left_stick_y) > 0.5;
            boolean down = (-gamepad1.left_stick_y) < -0.5;
            lift.update();
            if (up && !lastUp) {
                lift.up();
            }
            if (down && !lastDown) {
                lift.down();
            }
            if (gamepad1.x) servo.setPosition(0);
            else if (gamepad1.y) servo.setPosition(0.4);
            lastUp = up;
            lastDown = down;
        }
    }
}
