package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.runtime;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DetectPoleV2;
import org.firstinspires.ftc.teamcode.IOControl;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RR AUTO", group = "GearUp")
public class rrAutoComp3 extends LinearCleanupOpMode {
    public static final int[] VERTICAL_TARGETS = {20, 1450, 2200, 4500};
    public static DcMotor liftVertical1;

    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int CENTER = 2;
    int RIGHT = 3;
    String position = "";
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    Lift l;

    @Override
    public void cleanup() {
        if (frontLeft != null) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);
        }
        if (turret != null) {
            turret.setPower(0);
        }
        if (l != null && l.liftVertical1 != null) {
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            l.liftHorizontal.setPower(0);
        }
    }

    public void main() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // expose the hardware to the rest of the code (mainly 'turret' for now)
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        IOControl io = new IOControl(hardwareMap);
        DetectPoleV2 detector = new DetectPoleV2(turret, io.distSensorM, l, true);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        liftVertical1 = hardwareMap.get(DcMotor.class, "lift1");
        liftVertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftVertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(50);
        l.openClaw();

        camera.setPipeline(aprilTagDetectionPipeline);

        final boolean[] isCameraRunning = {false};
        final int[] cameraLoadAttempts = {1};
        final int warnAfter = 5;
        while (!isCameraRunning[0] && opModeInInit()) {
            stopMaybe();

            final boolean[] cameraFailed = {false};
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
                    isCameraRunning[0] = true;
                    telemetry.addLine("Camera is live.");
                    telemetry.update();
                }
                @Override
                public void onError(int errorCode) {
                    cameraFailed[0] = true;
                    cameraLoadAttempts[0]++;
                }
            });

            while (!(cameraFailed[0] || isCameraRunning[0]) && opModeInInit()) {
                stopMaybe();
                telemetry.addLine("Starting up camera: att " + cameraLoadAttempts[0]);
                if (cameraLoadAttempts[0] > warnAfter) {
                    telemetry.addLine("The robot is having trouble connecting");
                    telemetry.addLine("to the camera. Try restarting the OpMode,");
                    telemetry.addLine("or, if that doesn't work, restart the robot.");
                }
                telemetry.update();
            }
        }

        AprilTagDetection tagOfInterest = null;
        int targetLocation = 2;

        while (opModeInInit()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == CENTER || tag.id == RIGHT) {
                        if (tag.id == LEFT) {
                            targetLocation = 1;
                        }
                        if (tag.id == CENTER)
                            targetLocation = 2;
                        if (tag.id == RIGHT)
                            targetLocation = 3;
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("See a tag; id is " + tagOfInterest.id + " and our target is " + targetLocation);
                } else {
                    if (tagOfInterest == null) {
                        telemetry.addLine("Don't see a matching tag; no history; target is " + targetLocation);
                    } else {
                        telemetry.addLine("Don't see a matching tag; historical tag id is " + tagOfInterest.id + " and our target is " + targetLocation);
                    }
                }
            } else {
                if (tagOfInterest == null) {
                    telemetry.addLine("Don't see any tag; no history; target is " + targetLocation);
                } else {
                    telemetry.addLine("Don't see any tag; historical tag id is " + tagOfInterest.id + " and our target is " + targetLocation);
                }
            }
            telemetry.update();
            safeSleep(500);
        }

        waitForStart();
        l.closeClaw();
        safeSleep(250);
        if(position.equals("trig_left_side")){
            runtime.reset();
            int polePos = -400;

            //raises preloaded and drives to second tile, ready to drop off cone on pole
            l.verticalLift(2700, this);
            straight(0.6,54); // 54 function for driving straight

            //pole detect
            while (Math.abs(turret.getCurrentPosition()) < 700) {
                stopMaybe();
                if (io.distSensorM.getDistance(DistanceUnit.MM) <250 &&
                        turret.getCurrentPosition() < -380 && turret.getCurrentPosition() > -500) {
                    polePos = turret.getCurrentPosition();
                    break;
                }
                else if (turret.getCurrentPosition() < -480){
                    polePos = -420;
                    break;
                }
                turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turret.setPower(-0.25); //.35
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
            }

            turret.setPower(0);
            //sleep(300);

            telemetry.addData("polepos:", polePos);
            telemetry.update();

            //raises v lift to proper height above the pole
            runtime.reset();
            l.verticalLift(VERTICAL_TARGETS[3], this);
            while(l.liftVertical1.getCurrentPosition()< VERTICAL_TARGETS[3] && runtime.seconds()<1.5) { //1.8 seconds
                stopMaybe();
                l.update();
            }

            //drops off cone into the stack
            l.setHorizontalTargetManual(220);//208
            while (!l.isSatisfiedHorizontally()) {
                stopMaybe();
                l.update();
            }
            sleep(100);
            while(l.liftVertical1.getCurrentPosition()>3800){
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
            for (int i = 0; i < 4; i++) {
                //turns from pole to stack
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(ang); //750
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.8); //0.3
                sleep(500);

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
                sleep(300);

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
                l.setVerticalTargetManual(VERTICAL_TARGETS[3]-150);
                turr(-0.5, polePos + 37); //27

                //needs a little more juice at the top of pole to make it
                runtime.reset();
                while(runtime.seconds() < 0.5 + 0.12*i){
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.setHorizontalTargetManual(210); //225

                //once above pole, now we move downward to secure cone onto pole
                sleep(300);
                while(l.liftVertical1.getCurrentPosition()>3800){
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
        else if(position.equals("trig_right_side")){
            runtime.reset();

            int polePos = -400;

            //raises preloaded and drives to second tile, ready to drop off cone on pole
            l.verticalLift(2700, this);
            straight(0.6,54); // 54 function for driving straight

            //pole detect
            while (Math.abs(turret.getCurrentPosition()) >- 700) {
                stopMaybe();
                if (io.distSensorM.getDistance(DistanceUnit.MM) <250 &&
                        turret.getCurrentPosition() >365 && turret.getCurrentPosition() <500) {
                    polePos = turret.getCurrentPosition();
                    break;
                }
                else if (turret.getCurrentPosition() > 480){
                    polePos = 410;
                    break;
                }
                turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turret.setPower(0.20);
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
            }

            turret.setPower(0);
            telemetry.addData("polepos:", polePos);
            telemetry.update();

            //raises v lift to proper height above the pole
            runtime.reset();
            l.verticalLift(VERTICAL_TARGETS[3], this);
            while(l.liftVertical1.getCurrentPosition()< VERTICAL_TARGETS[3] && runtime.seconds()<1.5) { //1.8 seconds
                stopMaybe();
                l.update();
            }

            //drops off cone into the stack
            l.setHorizontalTargetManual(250);//240
            while (!l.isSatisfiedHorizontally()) {
                stopMaybe();
                l.update();
            }
            sleep(100);
            while(l.liftVertical1.getCurrentPosition()>3800){
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
            int ang = (int)TrigCalculations.stackAngleR((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
            int dist = (int)TrigCalculations.distToStack((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
            for (int i = 0; i < 4; i++) {
                //turns from pole to stack
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(-ang-15); //750
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.8); //0.3
                sleep(500);

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
                sleep(300);

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
                l.setVerticalTargetManual(VERTICAL_TARGETS[3]-150);
                turr(0.5, polePos -34); //37

                //needs a little more juice at the top of pole to make it
                runtime.reset();
                while(runtime.seconds() < 0.5 + 0.12*i){
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.setHorizontalTargetManual(235); //210

                //once above pole, now we move downward to secure cone onto pole
                sleep(300);
                while(l.liftVertical1.getCurrentPosition()>3800){
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
                turret.setPower(-0.3);
                sleep(800);
                l.setVerticalTargetManual(0);
                strafe(0.6, -21.5);
                //sleep(1500);
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
                turret.setPower(-0.3);
                sleep(800);
                l.setVerticalTargetManual(0);
                strafe(0.65, 22);
                //sleep(1500);
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
                turret.setPower(-0.3);
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
        else if (position.equals("left_side")) {
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
            l.setHorizontalTargetManual(250);//208
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
            int ang = (int)TrigCalculations.stackAngle((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
            int dist = (int)TrigCalculations.distToStack((encoderLeft.getCurrentPosition()+ encoderRight.getCurrentPosition())/2,encoderRear.getCurrentPosition());
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
                sleep(300);

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
                l.setHorizontalTargetManual(270); //225

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
                while(runtime.seconds() < 0.4){
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
        else if (position.equals("right_side")) {  // set by extending classes
            runtime.reset();
            int polePos = 370;

            //grabs pre-loaded
            l.closeClaw();
            sleep(500);

            //raises preloaded and drives to second tile, ready to drop off cone on pole
            l.verticalLift(2700, this);
            straight(0.5,54); //function for driving straight

            //pole detect
            while (io.distSensorM.getDistance(DistanceUnit.CM) > 250 && Math.abs(turret.getCurrentPosition()) < 700) {
                stopMaybe();
                turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                turret.setPower(0.25); //.35
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            turret.setPower(0);
            sleep(300);

            if (Math.abs(turret.getCurrentPosition()) > - 700)
                polePos = turret.getCurrentPosition();

            telemetry.addData("polepos:", polePos);
            telemetry.update();

            //raises v lift to proper height above the pole
            runtime.reset();
            l.verticalLift(VERTICAL_TARGETS[3], this);
            while(l.liftVertical1.getCurrentPosition()< VERTICAL_TARGETS[3] && runtime.seconds()<1.5) { //1.8 seconds
                stopMaybe();
                l.update();
            }

            //drops off cone into the stack
            l.setHorizontalTargetManual(215);//225
            while (!l.isSatisfiedHorizontally()) {
                stopMaybe();
                l.update();
            }
            sleep(100);
            while(l.liftVertical1.getCurrentPosition()>3800){
                stopMaybe();
                l.liftVertical1.setPower(-0.4);
                l.liftVertical2.setPower(-0.4);
            }

            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            l.openClaw();
            sleep(300);
            l.setHorizontalTarget(0);

            for (int i = 0; i < 3; i++) {
                //turns from pole to stack
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setTargetPosition(-780); //750
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.8); //0.3
                sleep(500);

                //lowers vertical lift to cone stack and extends out horizontal lift to stack
                l.setVerticalTargetManual(1100-i*90);
                while(l.liftVertical1.getCurrentPosition()>(1100-(i*90))){
                    stopMaybe();
                    l.liftVertical1.setPower(-0.6);
                    l.liftVertical2.setPower(-0.6);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.setHorizontalTargetManual(850);
                sleep(500); //1000
                l.closeClaw();
                sleep(300);

                //lifts cone off of stack and retracts h lift
                l.setVerticalTargetManual(1100 - (i * 90) + 250);
                runtime.reset();
                while(runtime.seconds()<0.5){
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);

                sleep(250);
                l.setHorizontalTargetManual(0);
                sleep(300);

                //extends v lift to height above the tall pole and rotates to it
                l.setVerticalTargetManual(VERTICAL_TARGETS[3]-150);
                turr(0.5, polePos - 41); //27

                //needs a little more juice at the top of pole to make it
                runtime.reset();
                while(runtime.seconds()<0.5+ 0.15*i){
                    stopMaybe();
                    l.liftVertical1.setPower(1);
                    l.liftVertical2.setPower(1);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.setHorizontalTargetManual(200); //225

                //once above pole, now we move downward to secure cone onto pole
                int c = 0;
                if(i ==2)
                    c =200;
                sleep(600 + c);
                while(l.liftVertical1.getCurrentPosition()>3800){
                    stopMaybe();
                    l.liftVertical1.setPower(-0.4);
                    l.liftVertical2.setPower(-0.4);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
                l.openClaw();
                l.setHorizontalTarget(0);
            }

            //resets turret and lift to home position, ready to be used in teleop, strafes to correct parking position based on what april tag position was detected
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if (targetLocation == 1) {
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.3);
                sleep(800);
                strafe(0.6, -18);
                //sleep(1500);
                while(l.liftVertical1.getCurrentPosition()>40) {
                    stopMaybe();
                    l.liftVertical1.setPower(-0.5);
                    l.liftVertical2.setPower(-0.5);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
            } else if (targetLocation == 3) {
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.3);
                straight(0.6, 60);
                sleep(800);
                strafe(0.6, 17);
                //sleep(1500);
                while(l.liftVertical1.getCurrentPosition()>40) {
                    stopMaybe();
                    l.liftVertical1.setPower(-0.5);
                    l.liftVertical2.setPower(-0.5);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
            }
            else{
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.3);
                sleep(800);
                runtime.reset();
                while(runtime.seconds() < 0.8){
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
                    l.liftVertical1.setPower(-0.5);
                    l.liftVertical2.setPower(-0.5);
                }
                l.liftVertical1.setPower(0);
                l.liftVertical2.setPower(0);
            }
            // while (opModeIsActive()) ; // wait for the match to end
        }

    }

    //--------------------------------------------------------------
    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    void turr(double speed, double position){
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        runtime.reset();
        if (position>0){
            while(turret.getCurrentPosition()<position){
                stopMaybe();
                if(turret.getCurrentPosition()>position-150){
                    turret.setPower(0.2);
                }
                else
                    turret.setPower(speed);
                l.update();
            }
        }
        else{
            while(turret.getCurrentPosition()>position){
                stopMaybe();
                if(turret.getCurrentPosition()<position+150){
                    turret.setPower(-0.2);
                }
                else
                    turret.setPower(speed);
                l.update();
            }
        }
        turret.setPower(0);
    }
    void straight(double speed, double distance) { // distance is in inches
        //1700 encoder counts to 1 inch.
        double adjust = 0.05;
        while ((encoderLeft.getCurrentPosition())/ (1700.0) <= distance) {
            stopMaybe();
            if(encoderLeft.getCurrentPosition()/1700>41){
                speed = 0.2;
            }

            if(encoderLeft.getCurrentPosition()> encoderRight.getCurrentPosition()){//power per inches 0.05 power per inch
                frontLeft.setPower(speed-adjust);
                frontRight.setPower(speed);
                rearLeft.setPower(speed-adjust);
                rearRight.setPower(speed);
            }
            else if(encoderLeft.getCurrentPosition()< encoderRight.getCurrentPosition()){//power per inches 0.05 power per inch
                frontLeft.setPower(speed+adjust);
                frontRight.setPower(speed);
                rearLeft.setPower(speed+adjust);
                rearRight.setPower(speed);
            }
            else if(encoderLeft.getCurrentPosition()== encoderRight.getCurrentPosition()){//power per inches 0.05 power per inch
                frontLeft.setPower(speed);
                frontRight.setPower(speed);
                rearLeft.setPower(speed);
                rearRight.setPower(speed);
            }
            l.update();


        }
        telemetry.addData("distance:", encoderLeft.getCurrentPosition()/1500.0);
        telemetry.update();
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);

    }
    void strafe(double speed, double distance) { // distance is in inches
        // 1700 encoder counts to 1 inch.
        // left = negative. checks distance in the correct direction.
        if(distance > 0){
            while ((encoderRear.getCurrentPosition() / 1700.0 <= distance)){
                stopMaybe();
                telemetry.addData("current data", encoderRear.getCurrentPosition()/ 1700.0);
                telemetry.addData("distance provided", distance);
                telemetry.update();

                frontLeft.setPower(speed);
                frontRight.setPower(-speed);
                rearLeft.setPower(-speed);
                rearRight.setPower(speed);
                l.update();
            }
        }
        else if(distance < 0){
            while ((encoderRear.getCurrentPosition() / 1700.0 >= distance)){
                stopMaybe();
                telemetry.addData("current data", encoderRear.getCurrentPosition()/ 1700.0);
                telemetry.addData("distance provided", distance);
                telemetry.update();

                frontLeft.setPower(-speed);
                frontRight.setPower(speed);
                rearLeft.setPower(speed);
                rearRight.setPower(-speed);
                l.update();
            }
        }

        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
    }

    public void driveStraight(double lf, double lb,
                              double rf, double rb, double distance) { //leftFront leftBack etc
        // sets power for all drive motors
        double posR = 0;
        double posL = 0;
        double mult = 1.0025;
        double mult2 = 1.0008;
        //encoderRight.setCurrentPosition() = 0;
        if (Math.abs(distance) < 8000) {
            mult = 1;
            mult2 = 1;
        }
        //robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (Math.abs((posR + posL) / 2) < Math.abs(distance)) {
            stopMaybe();
            posR = encoderRight.getCurrentPosition();
            posL = encoderLeft.getCurrentPosition();
            frontLeft.setPower(lf);
            rearLeft.setPower(lb);
            frontRight.setPower(rf);
            rearRight.setPower(rb);
            telemetry.addData("EncoderRight:", posR);
            telemetry.addData("Encoder Left:", posL);
            if (posR < posL) {
                rf = rf * mult;
                rb = rb * mult;
                lf = lf * mult2;
                lb = lb * mult2;
            } else {
                lf = lf * mult;
                lb = lb * mult;
                rf = rf * mult2; //mult2 is so that the robot doesn't become too tilted,
                rb = rb * mult2;
            }
            telemetry.update();
        }
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }
}
