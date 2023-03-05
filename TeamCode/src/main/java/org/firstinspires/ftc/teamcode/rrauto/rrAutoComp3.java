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
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DetectPoleV2;
import org.firstinspires.ftc.teamcode.IOControl;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RR AUTO", group = "GearUp")
public abstract class rrAutoComp3 extends LinearCleanupOpMode {
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
    IOControl io;

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
        // expose the hardware to the rest of the code (mainly 'turret' for now)
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        io = new IOControl(hardwareMap);
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
        main_auto_content(targetLocation); // haha abstraction go brrrr :L
    }

    //--------------------------------------------------------------
    @SuppressLint("DefaultLocale")
    void PIDTest(double distance, double speed){

        PID compensator = new PID(0);

        //1700 encoder counts to 1 inch.
        while ((encoderLeft.getCurrentPosition()) / (1700.0) <= distance) {
            double theta = compensator.calculateTheta(encoderRear.getCurrentPosition());
            double compensation = compensator.pidUpdate();

            telemetry.addLine(String.format("theta (deg): %.3f", Math.toDegrees(theta)));
            telemetry.addLine(String.format("compensation: %.3f ", compensation));
            telemetry.update();


            double currentPos = encoderLeft.getCurrentPosition() / 1700.0;
            double proportionTraveled = currentPos / distance;
            if(proportionTraveled > 0.1) {
                l.update();
                turrDrive(-250);
                if(proportionTraveled>0.7)
                    speed = 0.2;


            }

            frontLeft.setPower(speed + compensation);
            frontRight.setPower(speed);
            rearLeft.setPower(speed + compensation);
            rearRight.setPower(speed);
            //turrDrive(-250);
        }
        telemetry.speak("50");
        frontLeft.setPower(0);
        frontRight.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);


    }

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
                if(turret.getCurrentPosition()>position-300){
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
                if(turret.getCurrentPosition()<position+300){
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
            turrDrive(-250);


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
    public void turrDrive(int goal){
        if(turret.getCurrentPosition()>goal)
            turret.setPower(-0.2);
        else{
            turret.setPower(0);
        }
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

    abstract void main_auto_content(int targetLocation) throws InterruptedException;
}
