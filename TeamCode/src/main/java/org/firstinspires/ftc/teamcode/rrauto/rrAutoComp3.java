package org.firstinspires.ftc.teamcode.rrauto;

import static org.firstinspires.ftc.teamcode.SharedHardware.*;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

@Autonomous(name="RR AUTO" , group="GearUp")
public class rrAutoComp3 extends LinearOpMode {
    public static double SPEED = 40;
    public static double DIST_FIRST = 2;
    public static double DIST_SECOND = 4;
    public static double DIST_THIRD = 6;
    public DcMotor liftVertical1 = null;
    public DcMotor liftVertical2 = null;

    private ElapsedTime runtime = new ElapsedTime();

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

        /**
         * FIXME: why is this here?
         * @see org.firstinspires.ftc.teamcode.Lift#Lift(com.qualcomm.robotcore.hardware.HardwareMap)
         */
//        liftVertical1 = hardwareMap.get(DcMotor.class, "lift1");
//        liftVertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftVertical1.setTargetPosition(0);
//        liftVertical1.setDirection(DcMotorSimple.Direction.REVERSE);
//        liftVertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        liftVertical2 = hardwareMap.get(DcMotor.class, "lift2");
//        liftVertical2.setPower(0);
//        liftVertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        l.openClaw();
        waitForStart();
        if (position.equals("rrautotest")) {  // set by extending classes
            int a = 2; //counter for where to go
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
            l.closeClaw();
            sleep(250);
            l.speedVlift(1500);
           // l.setVerticalTargetManual(1500); // look, APIs exist for a reason (btw this one is new)

//            while (!l.isSatisfiedVertically()) {
//                telemetry.addLine("Waiting for lift...");
//                telemetry.addData("Current Position", l.liftVertical1.getCurrentPosition());
//                telemetry.addData("Goal", l.targetVerticalCount);
//                telemetry.update();
//                l.update();
//            }

//             TrajectorySequenceBuilder is better tbh -Miles TODO uncomment
            ArrayList<Trajectory> trags = new ArrayList<>();
            trags.add(drive.trajectoryBuilder(new Pose2d())
                    .forward(51,
                            SampleMecanumDrive.getVelocityConstraint(SPEED, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build());


            for (Trajectory t : trags) {
                drive.followTrajectory(t);
            }

            // NEW: Pole scanner
            detector.beginScan(DetectPoleV2.RotateDirection.CW);
            // DONE = done
            // IDLE = something broke :(
            while (detector.getState() != DetectPoleV2.State.DONE && opModeIsActive() && detector.getState() != DetectPoleV2.State.IDLE) {
                detector.run();
                l.update();
                telemetry.addData("Current state", detector.getState());
                telemetry.addData("Current reading", detector.lastDistance);
                telemetry.addData("Capture reading", detector.captureDistance);
                telemetry.update();
            }

            while (opModeIsActive()) ; // wait for the match to end
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
}
