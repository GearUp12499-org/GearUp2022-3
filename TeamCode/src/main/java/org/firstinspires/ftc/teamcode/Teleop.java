package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
            telemetry.update();
        }
    }

    //////////////////////////////////////////////////////////////////

    public void drive() {
        double speed = (0.25 + gamepad1.right_trigger * 0.75);
        double vX = 0; // forward/back
        double vY = 0; // left/right
        boolean useDPad = true;
        if (gamepad1.dpad_up) {
            vX += 1;
        } else if (gamepad1.dpad_down) {
            vX -= 1;
        } else if (gamepad1.dpad_left) {
            vY += 1;
        } else if (gamepad1.dpad_right) {
            vY -= 1;
        } else {
            useDPad = false;
        }
        if (useDPad) {
            double m1 = vX + vY;
            double m2 = vX - vY;
            double m3 = vX - vY;
            double m4 = vX + vY;
            driveMotor(speed * m1, speed * m2, speed * m3, speed * m4);
        } else {
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

    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    public void lift() {
        // TODO make dpad not go BRRRRRRRRRRRRRRRRRRRRRR
        if (gamepad2.y)
            l.setVerticalTarget(3);
        else if (gamepad2.b)
            l.setVerticalTarget(0);
        else if (gamepad2.a)
            l.setVerticalTarget(1);
        else if (gamepad2.x)
            l.setVerticalTarget(2);
        else if (gamepad2.dpad_up)
            l.moveVertical(10);
        else if (gamepad2.dpad_down)
            l.moveVertical(-10);

        if (gamepad2.right_bumper && !lastRightBumper) l.retract();
        else if (gamepad2.left_bumper && !lastLeftBumper) l.extend();
        else if (gamepad2.dpad_right) l.moveHorizontal(-10);
        else if (gamepad2.dpad_left) l.moveHorizontal(10);

        lastLeftBumper = gamepad2.left_bumper;
        lastRightBumper = gamepad2.right_bumper;

        // CLAW
        if (gamepad1.left_bumper) l.closeClaw();
        else if (gamepad1.right_bumper) l.openClaw();
        l.update();
    }

    ////////////////////////////////////////////////////////////////////

    public void turret() {
        if (l.liftVertical1.getCurrentPosition() < TURRET_THRESHOLD)
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
    }
}
