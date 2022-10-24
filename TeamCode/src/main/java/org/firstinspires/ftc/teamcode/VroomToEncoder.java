package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="run lift to encoder", group="!!!!!!!!")
public class VroomToEncoder extends LinearOpMode {
    boolean lastA = false;
    boolean lastB = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Lift lift = new Lift(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            lift.update();
            if (gamepad1.a && !lastA) {
                lift.up();
            }
            if (gamepad1.x && !lastB) {
                lift.down();
            }
            lastA = gamepad1.a;
            lastB = gamepad1.x;
        }
    }
}
