package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.nav.EncoderNavigation;
import org.firstinspires.ftc.teamcode.nav.Paths;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

/*
 * Because EasyOpenCv depends on OpenCV-Repackaged, you will also need to copy libOpenCvAndroid453.so
 * from the /doc/native_libs folder of that repo into the FIRST(sdcard/FIRST in Android Studio File Transfer) folder
 * on the USB storage of the Robot Controller (i.e. connect the Robot Controller to your computer
 * with a USB cable, put it into MTP mode, browse the contents of the file system, and drag 'n drop
 * the file into the "FIRST" folder at the top level of the filesystem).
 * Note that Control Hubs default to MTP mode and thus should be recognized immediately upon plugging
 * it in. For Mac, you will either need to use the Android File Transfer program,
 * or you can use the built-in file explorer side pane in Android Studio.
 * https://github.com/OpenFTC/EasyOpenCV
 * */

/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

@TeleOp
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode {

    public static final double SEC_PER_IN = (double)1/25;

    public void doTheMoveForwardThing(double inc) {  // i hate this so much
        double time = Math.abs(inc);
        int sig = (int) Math.signum(inc);
        frontLeft.setPower(0.5 * sig);
        rearLeft.setPower(0.5 * sig);
        rearRight.setPower(0.5 * sig);
        frontRight.setPower(0.5 * sig);
        sleep((long) (SEC_PER_IN * time * 1000));
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontRight.setPower(0);
    }

    public void doTheStrafeRightThing(double timing) {
//        for  bak
//        bak  for
        double time = Math.abs(timing);
        int sig = (int) Math.signum(timing);
        frontLeft.setPower(0.5 * sig);
        rearLeft.setPower(-0.5 * sig);
        rearRight.setPower(0.5 * sig);
        frontRight.setPower(-0.5 * sig);
        sleep((long) (time * 1000));
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        rearRight.setPower(0);
        frontRight.setPower(0);
    }

    public void zone1() {
        doTheMoveForwardThing(2);
        doTheStrafeRightThing(-1.75);
        doTheMoveForwardThing(34);
    }
    public void zone2() {
        doTheMoveForwardThing(36);
    }
    public void zone3() {
        doTheMoveForwardThing(2);
        doTheStrafeRightThing(1.75);
        doTheMoveForwardThing(34);
    }

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
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

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        prepareHardware(hardwareMap);
        Lift lift = new Lift(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

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
        lift.openClaw();

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == CENTER || tag.id == RIGHT) {
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

        lift.closeClaw();

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        telemetry.update();

        if (tagOfInterest == null || tagOfInterest.id == CENTER) {
            zone2();
        } else if (tagOfInterest.id == LEFT) {
            zone1();
        } else {
            zone3();
        }

        while (opModeIsActive()) {
            telemetry.addLine("done");
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}