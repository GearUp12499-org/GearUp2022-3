package org.firstinspires.ftc.teamcode.nav;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.NotImplemented;

@Autonomous(name = "Stupid Auto", group = "!!!!!!!!")
public class StupidAuto extends LinearOpMode {
    int target = 0;
    boolean lastA = false;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        prepareHardware(hardwareMap);
        EncoderNavigation nav = new EncoderNavigation(
                frontLeft, frontRight, rearLeft, rearRight, encoderLeft, encoderRight, encoderRear
        );
        Paths path = new Paths(nav);
        while (opModeInInit()) {
            telemetry.addData("Current Target", target + 1);
            telemetry.addLine("SMASH gp1 a button to change");
            telemetry.update();
            if (gamepad1.a && !lastA) {
                target ++;
                target %= 3;
            }
            lastA = gamepad1.a;
        }
        nav.moveForward(2);
        switch (target) {
            case 0:
                path.zone1();
                break;
            case 1:
                path.zone2();
                break;
            case 2:
                path.zone3();
                break;
            default:
                throw NotImplemented.i;
        }
        while (opModeIsActive()) {
            nav.asyncLoop();
            nav.dumpTelemetry(telemetry);
            telemetry.update();
        }
    }
}
