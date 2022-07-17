package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Run a Motor...")
public class RunAMotor extends LinearOpMode {

    double dir = 1;
    int which = 0;
    public static final int motor_count = 4;
    DcMotor target;


    public String numberToMotorName(int which_one) {
        switch (which_one) {
            case 0: return "frontLeft";
            case 1: return "frontRight";
            case 2: return "rearLeft";
            case 3: return "rearRight";
            default: return "invalid: " + which_one;
        }
    }


    public DcMotor numberToMotor(int which_one) {
        switch (which_one) {
            case 0: return frontLeft;
            case 1: return frontRight;
            case 2: return rearLeft;
            case 3: return rearRight;
            default: return null;
        }
    }

    boolean pushedA = false;
    boolean pushedB = false;

    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        while (!opModeIsActive()) {
            telemetry.addLine("Run a Motor");
            telemetry.addData("Switch Motor", "A");
            telemetry.addData("Switch Direction", "B");
            telemetry.addLine();
            telemetry.addData("Current Motor", numberToMotorName(which));
            telemetry.addData("Current Direction", dir == 1 ? "forward" : "backward");
            telemetry.update();
            if (gamepad1.a && !pushedA) {
                which ++;
                which %= motor_count;
            }
            if (gamepad1.b && !pushedB) {
                dir *= -1;
            }
            pushedA = gamepad1.a;
            pushedB = gamepad1.b;
        }
        target = numberToMotor(which);
        while (opModeIsActive()) {
            target.setPower(dir);
        }
        target.setPower(0);
    }
}
