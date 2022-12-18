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
        DcMotor turret = SharedHardware.turret;
        Lift l = new Lift(hardwareMap);
        l.closeClaw();

        waitForStart();
        l.setVerticalTarget(1);

        boolean ReachedVertTargetThree = false;
        while (opModeIsActive()) {
            if (!l.isSatisfiedVertically()) {
                l.update();
                telemetry.addLine("Waiting on lift... (" + l.targetVerticalCount + " -> " + l.liftVertical1.getCurrentPosition() + ")");
                telemetry.update();
                continue;
            }
            l.update();

            if (detector.getState() == DetectPoleV2.State.IDLE) {
                // if (gamepad2.x)
                detector.beginScan(DetectPoleV2.RotateDirection.CW);
            } else if (detector.getState() == DetectPoleV2.State.DONE) {
                l.setVerticalTarget(2);
                ReachedVertTargetThree = true;

                telemetry.addData("dis", detector.lastDistance);
                telemetry.update();
            }
            if (ReachedVertTargetThree && l.isSatisfiedVertically()) {
                l.openClaw();
                sleep(5000);

                int degFudge = 50;
                if (Math.abs(turret.getCurrentPosition()) < degFudge){
                    l.setVerticalTarget(0);
                    detector.stateChange(DetectPoleV2.State.IDLE);  // force reset
                    return;
                } else{
                    if(turret.getCurrentPosition() > 0){
                        turret.setTargetPosition(0);
                        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turret.setPower(-0.5);
                    }
                    else if(turret.getCurrentPosition() < 0){
                        turret.setTargetPosition(0);
                        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        turret.setPower(0.5);
                    }
                }

            }
            if (gamepad2.y) {
                SharedHardware.turret.setPower(gamepad2.left_stick_x);
            }
            if (isFallingEdge(gamepad2.y)) {
                SharedHardware.turret.setPower(0);
            }

            detector.run();
            telemetry.addLine("State be " + detector.getState());
            telemetry.addData("Reading", detector.lastDistance);
            telemetry.addData("Reached vert target 3", ReachedVertTargetThree);
            telemetry.update();
        }
    }
}