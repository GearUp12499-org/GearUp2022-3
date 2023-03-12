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
import static org.firstinspires.ftc.teamcode.rrauto.TrigCalculations.distToPoleMed;
import static org.firstinspires.ftc.teamcode.rrauto.TrigCalculations.poleAngle;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "left_side_medium", group = "Pushbot")
//@Disabled
public class left_side extends rrAutoComp3 {
    public static final double RATIO = (384.5 / 537.7);

    // Constructor
    public left_side() {
        position = "left_side";
    }

    @Override
    void main_auto_content(int targetLocation) throws InterruptedException {
        runtime.reset();
        int polePos = -1200;

        //raises preloaded and drives to second tile, ready to drop off cone on pole
        l.verticalLift((int)(RATIO*1300), this);
        PIDTest(51, 0.9, "left");

        //turr(-0.6, -180); // need to make this concurrent with lift and straight (is blocking rn) // 22.5deg * (750 / 90) = roughly 180
        //straight(0.6,54); // 54 function for driving straight // need to integrate id into pid (very inaccurate at 0.6->0.7 with only p)

        //pole detect
        while (Math.abs(turret.getCurrentPosition()) < 1350) {
            stopMaybe();
            if (io.distSensorM.getDistance(DistanceUnit.MM) <250 &&
                    turret.getCurrentPosition() < -1000 && turret.getCurrentPosition() > -1300) {
                polePos = turret.getCurrentPosition();
                break;
            }
            else if (turret.getCurrentPosition() < -900){
                //polePos = -(int)poleAngle((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
                polePos = -1200;
            }
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(turret.getCurrentPosition()>-780)
                turret.setPower(-0.55); //.35
            else{
                turret.setPower(-0.25);
            }

            telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        //polePos = -(int)poleAngle((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
        turret.setPower(0);
        l.verticalLift((int)(RATIO*VERTICAL_TARGETS[2]),this);
        while(turret.getCurrentPosition()<polePos){
            l.update();
            turret.setPower(0.2);
        }
        turret.setPower(0);

        telemetry.addData("polepos:", polePos);
        telemetry.update();

        //raises v lift to proper height above the pole
        runtime.reset();
        l.verticalLift((int)(RATIO*VERTICAL_TARGETS[2]), this);
        while(l.liftVertical1.getCurrentPosition()< 2400 ) { //1.8 seconds
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
        int distPole = (int)distToPoleMed((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());

        l.setHorizontalTargetManual(distPole+20);//208
       /* while (!l.isSatisfiedHorizontally()) {
            stopMaybe();
            l.update();
        }*/
        sleep(300);
        while(l.liftVertical1.getCurrentPosition()>1800){
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
        int ang = (int)TrigCalculations.stackAngle((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
        int dist = (int)TrigCalculations.distToStack((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
        int distPoleShort= (int)TrigCalculations.distToPoleMed((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
        int angShort= (int)TrigCalculations.poleAngleShort((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());

        for (int i = 0; i < 5; i++) {
            //turns from pole to stack
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setTargetPosition(ang); //750
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(1); //0.3
            if(i<3)
                sleep(1000);
            else
                sleep(300);
            //lowers vertical lift to cone stack and extends out horizontal lift to stack
            l.setVerticalTargetManual((int)(RATIO*1180-i*150));
            l.setHorizontalTargetManual(dist); //825
            while(l.liftVertical1.getCurrentPosition()>(1000-(i*150))){
                stopMaybe();
                l.liftVertical1.setPower(-0.6);
                l.liftVertical2.setPower(-0.6);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            if(i>2)
                sleep(300);
            //sleep(500); //1000
            l.closeClaw();
            sleep(350);

            //lifts cone off of stack and retracts h lift
            l.setVerticalTargetManual((int)(RATIO*(900 - (i * 135) + 250)));
            runtime.reset();
            while(runtime.seconds()<0.5){
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
            if(i <2) {
                l.setVerticalTargetManual((int)(RATIO*(VERTICAL_TARGETS[2]+1500)));

                turr(-0.6, polePos+10); //+15

                //needs a little more juice at the top of pole to make it
                runtime.reset();
                /*while (liftVertical1.getCurrentPosition() < 1700) {
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }*/
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.setHorizontalTargetManual(distPoleShort); //+15

                //once above pole, now we move downward to secure cone onto pole
                sleep(300);
                while (l.liftVertical1.getCurrentPosition() > 1800) {
                    stopMaybe();
                    l.liftVertical1.setPower(-0.4);
                    l.liftVertical2.setPower(-0.4);
                }
            }
            else{
                l.setVerticalTargetManual((int)(RATIO*(VERTICAL_TARGETS[1]+2500)));

                turr(0.6, 1150); //+15

                //needs a little more juice at the top of pole to make it
                runtime.reset();
                while (liftVertical1.getCurrentPosition() < 1400) {
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
        }

        //resets turret and lift to home position, ready to be used in teleop, strafes to correct parking position based on what april tag position was detected
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetLocation == 1) {
            l.setHorizontalTargetManual(0);

            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(-0.6);
            sleep(1000);
            l.setVerticalTargetManual((int)(RATIO*1000));
            strafe(0.6, -22.5);
            l.setVerticalTargetManual(0);
            while(l.liftVertical1.getCurrentPosition()>40) {
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
            turret.setPower(-0.6);
            sleep(800);
            l.setVerticalTargetManual((int)(RATIO*1000));
            strafe(0.65, 22);
            l.setVerticalTargetManual(0);
            while(l.liftVertical1.getCurrentPosition()>40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.3);
                l.liftVertical2.setPower(-0.3);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        }
        else{
            l.setHorizontalTargetManual(0);
            turret.setTargetPosition(0);
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(-0.6);
            sleep(1000);
            runtime.reset();
            while(runtime.seconds() < 0.4){
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
            while(l.liftVertical1.getCurrentPosition()>40) {
                stopMaybe();
                l.liftVertical1.setPower(-0.3);
                l.liftVertical2.setPower(-0.3);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        }
    }
}
