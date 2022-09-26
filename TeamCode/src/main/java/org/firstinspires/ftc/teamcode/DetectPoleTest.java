package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DetectPoleTest", group = "!!!!!!!!")
public class DetectPoleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SharedHardware.prepareHardware(hardwareMap);
        IOControl io = new IOControl(hardwareMap);
        DetectPoleOneSensor dpos = new DetectPoleOneSensor(SharedHardware.turret, io.distSensorM);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("State", dpos.state);
            telemetry.addData("Last Distance", dpos.lastDist);
            telemetry.update();
            if ((dpos.state == DetectPoleOneSensor.State.FINISH
                    || dpos.state == DetectPoleOneSensor.State.FAILED)) {
                if (gamepad2.x) {
                    dpos.beginScan();
                }
                SharedHardware.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                if (gamepad2.y) {
                    SharedHardware.turret.setPower(gamepad2.left_stick_x);
                } else {
                    SharedHardware.turret.setPower(0);
                }
            }
            dpos.update();
        }
    }
}
