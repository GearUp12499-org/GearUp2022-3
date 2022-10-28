package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp")
public class Teleop extends LinearOpMode {
    public Lift l;

    public final int TURRET_THRESHOLD = 800;
    public final int TURRET_DELTA = 1500; // STILL HAVE TO TEST
    public int turret_center;

    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        turret_center = turret.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()) {
            drive();
            lift();
            turret();
        }
    }

    //////////////////////////////////////////////////////////////////

    public void drive() {
        double speed = (0.25 + gamepad1.left_trigger * 0.75);
        if (gamepad1.dpad_up)
            driveMotor(speed, speed, speed, speed);
        else if (gamepad1.dpad_down)
            driveMotor(-speed, -speed, -speed, -speed);
        else if (gamepad1.dpad_left)
            driveMotor(-speed, speed, speed, -speed);
        else if (gamepad1.dpad_right)
            driveMotor(speed, -speed, -speed, speed);
        else {
            double left = -gamepad1.left_stick_y,
                    right = -gamepad1.right_stick_y;
            driveMotor(left, left, right, right);
        }
    }

    public void driveMotor(double lf, double lb, double rf, double rb) {
        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        rearLeft.setPower(lb);
        rearRight.setPower(rb);
    }

    /////////////////////////////////////////////////////

    public void lift() {
        if (gamepad2.y)
            l.setTarget(3);
        else if (gamepad2.b)
            l.setTarget(0);
        else if (gamepad2.a)
            l.setTarget(1);
        else if (gamepad2.x)
            l.setTarget(2);
        else if (gamepad2.dpad_up)
            l.move(10);
        else if (gamepad2.dpad_down)
            l.move(-10);

        /*
        telemetry.addData("targetCount", l.targetCount);
        telemetry.addData("current count:", l.l1.getCurrentPosition());
        // telemetry.addData("l1 power:", l.l1.getPower());
        telemetry.update();
        */

        l.update();
    }

    ////////////////////////////////////////////////////////////////////

    public void turret() {
        if (l.l1.getCurrentPosition() < TURRET_THRESHOLD)
            return;

        double speed = gamepad2.left_stick_x * 0.25;
        int now = turret.getCurrentPosition() - turret_center;
        if ((speed < 0 && now > -TURRET_DELTA) || (speed > 0 && now < TURRET_DELTA))
            turret.setPower(speed);
        else
            turret.setPower(0);

        telemetry.addData("turret center:", turret_center);
        telemetry.addData("turret position:", now);
        telemetry.addData("turret speed:", speed);
        telemetry.update();
    }
}
