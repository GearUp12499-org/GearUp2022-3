package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "LiftMotorTest")
public class LiftMotorTest extends LinearOpMode {
    public Lift l;

    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        l.liftVertical1.setPower(0);
        l.liftVertical2.setPower(0);

        waitForStart();
        while (opModeIsActive()) {
            lift();

            telemetry.update();
        }
    }

    public void lift() {
        l.liftVertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        l.liftVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double currentSpeed = Lift.POWER_UP;
        if (gamepad1.y) {
            while (l.liftVertical1.getCurrentPosition() < 4000) {
                l.liftVertical1.setPower(currentSpeed);
                l.liftVertical2.setPower(currentSpeed);
                if(l.liftVertical1.getCurrentPosition()>3750){
                    l.liftVertical1.setPower(currentSpeed/2);
                    l.liftVertical2.setPower(currentSpeed/2);
                }

            }
            l.liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l.liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            l.liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l.liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l.liftVertical2.setPower(0);
            l.liftVertical1.setPower(0);
        }
        telemetry.addData("lv1", l.liftVertical1.getCurrentPosition());
        telemetry.addData("lv2", l.liftVertical2.getCurrentPosition());
        //telemetry.addData('turret', turret.getCurrentPosition)
    }
}
