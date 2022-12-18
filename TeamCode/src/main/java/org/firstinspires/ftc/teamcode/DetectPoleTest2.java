package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.RisingFallingEdges.isFallingEdge;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DetectPoleTest2", group = "!!!!!!!!")
public class DetectPoleTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SharedHardware.prepareHardware(hardwareMap);
        IOControl io = new IOControl(hardwareMap);
        DetectPoleV2 detector = new DetectPoleV2(SharedHardware.turret, io.distSensorM);
        Lift l = new Lift(hardwareMap);
        l.closeClaw();
        waitForStart();
        l.setVerticalTarget(2);
        int kineState = 0;
        while (opModeIsActive()) {
            if (!l.isSatisfiedVertically()) {
                l.update();
                telemetry.addLine("Waiting on lift... (" + l.targetVerticalCount  + " -> " + l.liftVertical1.getCurrentPosition() + ")");
                telemetry.update();
                continue;
            }
            l.update();
            if (detector.getState() == DetectPoleV2.State.IDLE) {
                if (gamepad2.x) detector.beginScan(DetectPoleV2.RotateDirection.CW);
            } else if (detector.getState() == DetectPoleV2.State.DONE) {
                kineState = 1;
                l.setVerticalTarget(3);
            }
            if (kineState == 1 && l.isSatisfiedVertically()) {
                kineState = 2;
                l.openClaw();
                detector.stateChange(DetectPoleV2.State.IDLE);  // force reset
            }
            if (gamepad2.y) {
                SharedHardware.turret.setPower(gamepad2.left_stick_x * 0.5);
            }
            if (isFallingEdge(gamepad2.y)) {
                SharedHardware.turret.setPower(0);
            }
            detector.run();
            telemetry.addLine("State be " + detector.getState());
            telemetry.addData("Reading", detector.lastDistance);
            telemetry.addData("Kine state", kineState);
            telemetry.update();
        }
    }
}
