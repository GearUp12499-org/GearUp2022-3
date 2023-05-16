package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.runtime;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;
import static org.firstinspires.ftc.teamcode.rrauto.TrigCalculations.distToPoleHigh;
import static org.firstinspires.ftc.teamcode.rrauto.TrigCalculations.poleAngle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.DurationFormatter;


@Autonomous(name = "trig_right_side", group = "Pushbot")
//@Disabled
public class trig_right_side extends rrAutoComp3 {

    // Constructor
    public trig_right_side() {
        position = "trig_right_side";
    }

    @Override
    void main_auto_content(int targetLocation) throws InterruptedException {
        runtime.reset();
        int polePos = -400;

        //raises preloaded and drives to second tile, ready to drop off cone on pole
        l.verticalLift(3300, this); //3200
        PIDTest(50, 0.9, "right");

        //turr(-0.6, -180); // need to make this concurrent with lift and straight (is blocking rn) // 22.5deg * (750 / 90) = roughly 180
        //straight(0.6,54); // 54 function for driving straight // need to integrate id into pid (very inaccurate at 0.6->0.7 with only p)

        //pole detect
        boolean useTrig = true;
        while (Math.abs(turret.getCurrentPosition()) < 700) {
            stopMaybe();
            if (io.distSensorM.getDistance(DistanceUnit.MM) < 250 &&
                    turret.getCurrentPosition() > 300 && turret.getCurrentPosition() < 700) {
                polePos = turret.getCurrentPosition();
                persistTeleData.put("used strategy", "pole detection @ " + polePos);
                useTrig = false;
                break;
            }
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (turret.getCurrentPosition() < 180)
                turret.setPower(0.34); //.35
            else {
                turret.setPower(0.15);
            }

            telemetry.addData("distance", io.distSensorM.getDistance(DistanceUnit.CM));
            telemetry.addData("height", l.targetVerticalCount);
            telemetry.update();
            l.update();
        }
//        if (useTrig) {
        polePos = (int) poleAngle((encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2, encoderRear.getCurrentPosition());
        persistTeleData.put("used strategy", "trig @ " + polePos);
//        }
        persistTeleData.put("detect", "all done " + DurationFormatter.formatDuration(entireRun.milliseconds()));
//        polePos = -(int)poleAngle((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
        turret.setPower(0);
        l.verticalLift(VERTICAL_TARGETS[3], this); //1500
        while (turret.getCurrentPosition() > polePos) {
            l.update();
            turret.setPower(-0.1);
        }
        turret.setPower(0);

        telemetry.addData("polepos:", polePos);
        telemetry.update();

        //raises v lift to proper height above the pole
        runtime.reset();
        l.verticalLift(VERTICAL_TARGETS[3], this); //1500
        while (l.liftVertical1.getCurrentPosition() < 3300) { //3300
            stopMaybe();
            //l.update();
            l.liftVertical1.setPower(1);
            l.liftVertical2.setPower(1);
        }
        runtime.reset();
       /* while(runtime.seconds() < 0.2 ){
            stopMaybe();
            l.liftVertical1.setPower(1);
            l.liftVertical2.setPower(1);
        }*/
        l.liftVertical1.setPower(0);
        l.liftVertical2.setPower(0);
        //drops off cone into the stack
        int distPole = (int) distToPoleHigh((encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2, encoderRear.getCurrentPosition());

        l.setHorizontalTargetManual(distPole + 20);//208
       /* while (!l.isSatisfiedHorizontally()) {
            stopMaybe();
            l.update();
        }*/
        sleep(300);
        while (l.liftVertical1.getCurrentPosition() > 3800 * (384.5 / 537.7)) {
            stopMaybe();
            l.liftVertical1.setPower(-0.4);
            l.liftVertical2.setPower(-0.4);
        }

        l.liftVertical1.setPower(0);
        l.liftVertical2.setPower(0);
        l.openClaw();
        sleep(150);
        l.setHorizontalTarget(0);
        telemetry.addData("robot x pos:", encoderLeft.getCurrentPosition());
        telemetry.addData("robot y pos:", encoderRear.getCurrentPosition());

        telemetry.update();
        RobotLog.i("xArgs " + (encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2);
        RobotLog.i("yArgs " + encoderRear.getCurrentPosition());
        int ang = -(int) TrigCalculations.stackAngle((encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2, encoderRear.getCurrentPosition());
        int dist = (int) TrigCalculations.distToStack((encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2, encoderRear.getCurrentPosition());
        int distPoleShort = (int) TrigCalculations.distToPoleMed((encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2, encoderRear.getCurrentPosition());
        int angShort = -(int) TrigCalculations.poleAngleShort((encoderLeft.getCurrentPosition() + encoderRight.getCurrentPosition()) / 2, encoderRear.getCurrentPosition());
        RobotLog.i("the angShort is " + angShort + ", or " + angShort / TrigCalculations.encToAngle);

        for (int i = 0; i < 5; i++) {
            //turns from pole to stack
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setTargetPosition(ang); //750
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(1); //0.3
            sleep(500);

            //lowers vertical lift to cone stack and extends out horizontal lift to stack
            l.setVerticalTargetManual(1180 - i * 150);
            l.setHorizontalTargetManual(dist); //825
            while (l.liftVertical1.getCurrentPosition() > (900 - (i * 150))) {
                stopMaybe();
                l.liftVertical1.setPower(-0.6);
                l.liftVertical2.setPower(-0.6);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            //sleep(500); //1000
            l.closeClaw();
            sleep(350);

            //lifts cone off of stack and retracts h lift
            l.setVerticalTargetManual(900 - (i * 135) + 250);
            runtime.reset();
            while (runtime.seconds() < 0.37) {
                stopMaybe();
                l.liftVertical1.setPower(1);
                l.liftVertical2.setPower(1);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);

            //sleep(250);
            l.setHorizontalTargetManual(0);
            sleep(300);

            //extends v lift to height above the tall pole and rotates to it
            if (i < 4) {
                l.setVerticalTargetManual(VERTICAL_TARGETS[3] - 1000); //1500

                turr(0.6, polePos); //+15

                //needs a little more juice at the top of pole to make it
                runtime.reset();
                while (liftVertical1.getCurrentPosition() < 3200) {//3300
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.setHorizontalTargetManual(distPole); //+15

                //once above pole, now we move downward to secure cone onto pole
                sleep(300);
                while (l.liftVertical1.getCurrentPosition() > 3800 * (384.5 / 537.7)) {
                    stopMaybe();
                    l.liftVertical1.setPower(-0.4);
                    l.liftVertical2.setPower(-0.4);
                }
            } else {
                l.setVerticalTargetManual(VERTICAL_TARGETS[1] + 1500);
                turr(-0.6, angShort); //+15

                //needs a little more juice at the top of pole to make it
                runtime.reset();
                while (liftVertical1.getCurrentPosition() < 800) {
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.setHorizontalTargetManual(distPoleShort); //+15

                //once above pole, now we move downward to secure cone onto pole
                sleep(300);
                while (l.liftVertical1.getCurrentPosition() > 1000 * (384.5 / 537.7)) {
                    stopMaybe();
                    l.liftVertical1.setPower(-0.4);
                    l.liftVertical2.setPower(-0.4);
                }
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            l.openClaw();
            sleep(150);
            l.setHorizontalTarget(0);
            sleep(100);

            persistTeleData.put("additional cone #" + (i + 1), DurationFormatter.formatDuration(entireRun.milliseconds()));
            telemetry.update();
        }

        //resets turret and lift to home position, ready to be used in teleop, strafes to correct parking position based on what april tag position was detected
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetLocation == 1) {
            l.setHorizontalTargetManual(0);

            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.6);
            sleep(1000);
            l.setVerticalTargetManual(1000);
            strafe(0.6, -22.5);
            l.setVerticalTargetManual(0);
            while (l.liftVertical1.getCurrentPosition() > 40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.3);
                l.liftVertical2.setPower(-0.3);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        } else if (targetLocation == 3) {
            l.setHorizontalTargetManual(0);

            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.6);
            sleep(800);
            l.setVerticalTargetManual(1000);
            strafe(0.65, 22);
            l.setVerticalTargetManual(0);
            while (l.liftVertical1.getCurrentPosition() > 40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.3);
                l.liftVertical2.setPower(-0.3);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        } else {
            l.setHorizontalTargetManual(0);
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.6);
            sleep(1000);
            runtime.reset();
            while (runtime.seconds() < 0.4) {
                stopMaybe();
                frontLeft.setPower(-0.4);
                frontRight.setPower(-0.4);
                rearLeft.setPower(-0.4);
                rearRight.setPower(-0.4);
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
            while (l.liftVertical1.getCurrentPosition() > 40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.3);
                l.liftVertical2.setPower(-0.3);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        }

        persistTeleData.put("all done", DurationFormatter.formatDuration(entireRun.milliseconds()));
        telemetry.update();
    }
}
