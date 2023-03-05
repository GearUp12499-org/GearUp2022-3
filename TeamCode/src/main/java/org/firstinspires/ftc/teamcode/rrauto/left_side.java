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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "left_side_medium", group = "Pushbot")
//@Disabled
public class left_side extends rrAutoComp3 {

    // Constructor
    public left_side() {
        position = "left_side";
    }

    @Override
    void main_auto_content(int targetLocation) throws InterruptedException {
        runtime.reset();
        int polePos = -1200;

        //raises preloaded and drives to second tile, ready to drop off cone on pole
        l.verticalLift(2200, this);
        straight(0.6,54); // 54 function for driving straight

        //pole detect
        while (Math.abs(turret.getCurrentPosition()) < 1400) {
            stopMaybe();
            if (io.distSensorM.getDistance(DistanceUnit.MM) < 300 &&
                    turret.getCurrentPosition() < -1000 && turret.getCurrentPosition() > -1300) {
                polePos = turret.getCurrentPosition();
                break;
            }
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setPower(-0.30); //.25
            telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
            telemetry.update();
        }

        int ang = (int)TrigCalculations.stackAngle((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
        int dist = (int)TrigCalculations.distToStack((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
        int distPole = (int)TrigCalculations.distToPoleMed((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());

        turret.setPower(0);
        //sleep(300);

        telemetry.addData("polepos:", polePos);
        telemetry.update();

        //raises v lift to proper height above the pole
        runtime.reset();
        l.verticalLift(3500, this);
        while(l.liftVertical1.getCurrentPosition()< 3500 && runtime.seconds()<1.5) { //1.8 seconds
            stopMaybe();
            l.update();
        }

        //drops off cone into the stack
        l.setHorizontalTargetManual(distPole);//208
        while (!l.isSatisfiedHorizontally()) {
            stopMaybe();
            l.update();
        }
        sleep(100);
        while(l.liftVertical1.getCurrentPosition()>2800){
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
        for (int i = 0; i < 3; i++) {
            //turns from pole to stack
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turret.setTargetPosition(ang); //750
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turret.setPower(0.5); //0.8
            sleep(1600);

            //lowers vertical lift to cone stack and extends out horizontal lift to stack
            l.setVerticalTargetManual(1180-i*150);
            l.setHorizontalTargetManual(dist); //825
            while(l.liftVertical1.getCurrentPosition()>(1180-(i*150))){
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
            l.setVerticalTargetManual(1180 - (i * 150) + 250);
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
            l.setVerticalTargetManual(3600);
            turr(-0.4, polePos); //0.5

            //needs a little more juice at the top of pole to make it
            runtime.reset();
                /*while(runtime.seconds() < 0.5 + 0.12*i){
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }*/
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            l.setHorizontalTargetManual(distPole); //270

            //once above pole, now we move downward to secure cone onto pole
            sleep(500);
            while(l.liftVertical1.getCurrentPosition()>2800){
                stopMaybe();
                l.liftVertical1.setPower(-0.4);
                l.liftVertical2.setPower(-0.4);
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
            turret.setPower(0.3);
            sleep(800);
            l.setVerticalTargetManual(0);
            strafe(0.6, -22.5);
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
            turret.setPower(0.3);
            sleep(800);
            runtime.reset();
            while(runtime.seconds() < 0.3){
                stopMaybe();
                frontLeft.setPower(0.4);
                frontRight.setPower(0.4);
                rearLeft.setPower(0.4);
                rearRight.setPower(0.4);
            }
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
            l.setVerticalTargetManual(0);
            strafe(0.65, 22);
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
            turret.setPower(0.3);
            sleep(800);
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
                l.liftVertical1.setPower(-0.8);
                l.liftVertical2.setPower(-0.8);
            }
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
        }
    }
}
