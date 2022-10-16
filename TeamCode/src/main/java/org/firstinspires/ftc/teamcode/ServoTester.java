package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@TeleOp(name="Servo tester", group="!!!!!!!!")
public class ServoTester extends LinearOpMode {
    boolean lastA = false;
    static final double low = 0;
    static final double hi = 0.4;

    ElapsedTime runtime = new ElapsedTime();

    double target = low;
    private boolean lastB = false;
    private int bc = 0;
    private int ac = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        runtime.reset();
        servo.setPosition(target);
        while (opModeIsActive()) {
            if (gamepad1.b) {
                telemetry.addLine("Updating...");
                telemetry.update();
                target = low;
                servo.setPosition(target);
                if (!lastB) bc ++;
            }
            if (gamepad1.a) {
                telemetry.addLine("Updating...");
                telemetry.update();
                target = hi;
                servo.setPosition(target);
                if (!lastA) ac ++;
            }
            telemetry.addData("current target", target);
            telemetry.addData("t", runtime.seconds());
            telemetry.addData("bc", bc + " <" + lastB + ">");
            telemetry.addData("ac", ac + " <" + lastA + ">");
            telemetry.addLine(String.format(Locale.ENGLISH, "Press A (gp1) to switch %f/%f", low, hi));
            telemetry.update();
            lastA = gamepad1.a;
            lastB = gamepad1.b;
        }
    }
}
