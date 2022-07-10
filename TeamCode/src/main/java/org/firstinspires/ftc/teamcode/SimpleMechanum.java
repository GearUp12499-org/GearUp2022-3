package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Testing")
public class SimpleMechanum extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor rearLeft;
    private DcMotor rearRight;

    private void prepareHardware() {
        // TODO put actual hardware names
        frontLeft = hardwareMap.get(DcMotor.class, "front_left");
        frontRight = hardwareMap.get(DcMotor.class, "front_right");
        rearLeft = hardwareMap.get(DcMotor.class, "rear_left");
        rearRight = hardwareMap.get(DcMotor.class, "rear_right");
    }

    @Override
    public void runOpMode() {
        // tank
        waitForStart();
        frontLeft.setPower(1);
        frontRight.setPower(1);
        rearLeft.setPower(1);
        rearRight.setPower(1);
        sleep(1000);
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }
}
