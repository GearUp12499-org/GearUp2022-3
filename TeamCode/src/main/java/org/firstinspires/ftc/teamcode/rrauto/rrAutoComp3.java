package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.SharedHardware.liftVertical1;

import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.runtime;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;


import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DetectPoleV2;
import org.firstinspires.ftc.teamcode.IOControl;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "RR AUTO", group = "GearUp")
public class rrAutoComp3 extends LinearOpMode {
    public static final int[] VERTICAL_TARGETS = {20, 1450, 2200, 4500};
    public static DcMotor liftVertical1;

    //public DcMotor liftVertical1 = null;
    public DcMotor liftVertical2 = null;

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
    AprilTagDetection tagOfInterest = null;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    Lift l;

    @Override
    public void runOpMode() throws InterruptedException {
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
        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.setMsTransmissionInterval(50);
        l.openClaw();
        waitForStart();
        if (position.equals("rrautotest")) {  // set by extending classes
            int a = 2; //counter for where to go
            /*
            runtime.reset();
            while (runtime.seconds()<0.5) {
                ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
                if (currentDetections.size() != 0) {
                    boolean tagFound = false;
                    for (AprilTagDetection tag : currentDetections) {
                        if (tag.id == LEFT || tag.id == CENTER || tag.id == RIGHT) {
                            if(tag.id == LEFT){
                                a =1;
                            }
                            if(tag.id == CENTER)
                                a =2;
                            if(tag.id == RIGHT)
                                a =3;
                            tagOfInterest = tag;
                            tagFound = true;
                            break;
                        }
                    }
                    if (tagFound) {
                        telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                        tagToTelemetry(tagOfInterest);
                    } else {
                        telemetry.addLine("Don't see tag of interest :(");
                        if (tagOfInterest == null) {
                            telemetry.addLine("(The tag has never been seen)");
                        } else {
                            telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                            tagToTelemetry(tagOfInterest);
                        }
                    }
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
                telemetry.update();
                sleep(20);
            }
*/
            int polePos = -370;

            //grabs pre-loaded
            l.closeClaw();
            sleep(500);

            //raises preloaded and drives to second tile, ready to drop off cone on pole
            while(l.liftVertical1.getCurrentPosition()<2700) { //this while loop will no longer be needed after testing, since update is within straight
                l.verticalLift(2700, this); //is now going to close to the pole at the top because the lift seems pretty stable and it reduces time
                l.update();
            }
            //straight(0.5,54); //function for driving straight

            //pole detect
            while (io.distSensorM.getDistance(DistanceUnit.CM) > 250 && Math.abs(turret.getCurrentPosition()) < 700) {
                turret.setPower(-0.35); //.35
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            turret.setPower(0);

            if (Math.abs(turret.getCurrentPosition()) < 700)
                polePos = turret.getCurrentPosition();

            telemetry.addData("polepos:", polePos);
            telemetry.update();

            //raises v lift to proper height above the pole
            while(l.liftVertical1.getCurrentPosition()< VERTICAL_TARGETS[3]) { //1.8 seconds
                l.verticalLift(VERTICAL_TARGETS[3], this);
                l.update();
            }

            //drops off cone into the stack
            l.setHorizontalTargetManual(215);//225
            while (!l.isSatisfiedHorizontally()) l.update();
            sleep(100);
            l.openClaw();
            sleep(300);
            l.setHorizontalTarget(0);

            for (int i = 0; i < 1; i++) {
                //turns from pole to stack
                turret.setTargetPosition(740); //750
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.8); //0.3
                sleep(500);

                //lowers vertical lift to cone stack and extends out horizontal lift to stack
                l.setVerticalTargetManual(1100-i*90);
                while(l.liftVertical1.getCurrentPosition()<(1100-(i*90)))
                    l.update();
                l.setHorizontalTargetManual(850);
                sleep(500); //1000
                l.closeClaw();
                sleep(300);

                //lifts cone off of stack and retracts h lift
                l.setVerticalTargetManual(1100 - (i * 90) + 250);
                while(l.liftVertical1.getCurrentPosition()<(1100-(i*90)+200))
                    l.update();
                l.setHorizontalTargetManual(0);
                sleep(300);

                //extends v lift to height above the tall pole and rotates to it
                l.setVerticalTargetManual(VERTICAL_TARGETS[3]);
                while(l.liftVertical1.getCurrentPosition()<VERTICAL_TARGETS[3])
                    l.update();
                turret.setTargetPosition(polePos + 27);
                turret.setPower(-0.5); //0.3

                //drops off cone onto pole
                sleep(2250);
                //l.update();
                l.setHorizontalTargetManual(225); //225
                sleep(350); //650
                l.openClaw();
                sleep(300);
                l.setHorizontalTarget(0);
            }
/*
            //resets turret and lift to home position, ready to be used in teleop
            turret.setTargetPosition(0);
            turret.setPower(1); //0.3

            sleep(500);
            l.verticalLift(VERTICAL_TARGETS[0], this);
            */
            //strafes to correct parking position based on what april tag position was detected
            a =2;
            if (a == 1) {
                strafe(0.6, 'l', 17);
            } else if (a == 3) {
                strafe(0.6, 'r', 17);
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
    void straight(double speed, double distance) { // distance is in inches
        //1700 encoder counts to 1 inch.
        double adjust = 0.05;
        while ((encoderLeft.getCurrentPosition())/ (1700.0) <= distance) {
            if(encoderLeft.getCurrentPosition()/1700>45){
                frontLeft.setPower(0.2);
                frontRight.setPower(0.2);
                rearLeft.setPower(0.2);
                rearRight.setPower(0.2);
            }
            else if(encoderLeft.getCurrentPosition()> encoderRight.getCurrentPosition()){//power per inches 0.05 power per inch
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
    void strafe(double speed, char direction, double distance) { // distance is in inches
        //1700 encoder counts to 1 inch.
        while (encoderRear.getCurrentPosition() / 1700.0 <= distance) {
            if (direction == 'l') {
                frontLeft.setPower(-speed);
                frontRight.setPower(speed);
                rearLeft.setPower(speed);
                rearRight.setPower(-speed);
            } else if (direction == 'r') {
                frontLeft.setPower(speed);
                frontRight.setPower(-speed);
                rearLeft.setPower(-speed);
                rearRight.setPower(speed);
            }

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
