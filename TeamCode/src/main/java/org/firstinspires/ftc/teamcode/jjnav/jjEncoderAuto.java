package org.firstinspires.ftc.teamcode.jjnav;

import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderRight;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.nav.EncoderNavigation;
import org.firstinspires.ftc.teamcode.nav.Paths;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.rrauto.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderRight;

@Autonomous(name="GearUp: Drive Autonomous" , group="GearUp")
//@Disabled
public abstract class jjEncoderAuto extends LinearOpMode {
    /* Declare OpMode members. */
    GearUpHardware          robot   = new GearUpHardware();   // Use GearUp hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV    = 537;    // eg: Neverest 40
    static final double DRIVE_GEAR_REDUCTION    = 0.707 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416);

    int             target = 0;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;

    String alliance = "";
    String position = "";
    int startPosition = 0 ;
    // Positions
    int liftZero = 0;
    int liftBot = 220;
    int liftMid = 311;
    int liftTop = 500;

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
    //l = new Lift(hardwareMap);
    // IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    //Vuforia

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        // set up dc motors

        // Send telemetry message to indicate successful Encoder reset
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

        waitForStart();

//----------------------------------------------------------------------------------------------------------------
        if (position.equals("encoderDriveTest")){
           /* runtime.reset();
            while(runtime.seconds()< 2.5) {
                robot.vLiftLeft.setTargetPosition(800);
                robot.vLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vLiftLeft.setPower(-1);
                //robot.vLiftRight.setPower(-1);
            }*/

            int a = 2; //counter for where to go
            runtime.reset();
           while (runtime.seconds()<3) {
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



            robot.servo.setPosition(.52);
            sleep(2000);
            runtime.reset();
            while(runtime.seconds()< 2.5) {
                robot.vLiftLeft.setTargetPosition(800);
                robot.vLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vLiftLeft.setPower(-1);
                //robot.vLiftRight.setPower(-1);
            }
            sleep(3000);
            robot.vLiftRight.setPower(0);
            driveStraight(0.3,0.3,0.3,0.3, 5000);
            //robot.rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            if(a ==1) {
                driveStrafe(0.3, 1, 44000);
                driveStraight(0.3, 0.3, 0.3, 0.3, 49000);
            }
            else if(a == 3) {
                runtime.reset();
                //while(runtime.seconds()<3)
                    driveStrafe(0.3, -1, 45500);
                driveStraight(0.3, 0.3, 0.3, 0.3, 49000); //commented out to test lift/gripper, but this is right distance from wall to center of second tile.
            }
            else{
                driveStraight(0.3, 0.3, 0.3, 0.3, 53000); //commented out to test lift/gripper, but this is right distance from wall to center of second tile.

            }
            robot.leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
           // robot.rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            //robot.leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


            sleep(200);
            runtime.reset();
            if (a ==2) {
                while (runtime.seconds() < 2.25)
                    driveStraight(-0.5, -0.5, -0.5, -0.5, -7000);
            }

            runtime.reset();
            robot.leftFront.setPower(0);

            robot.leftBack.setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack.setPower(0);
            robot.vLiftLeft.setTargetPosition(0);
            robot.vLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vLiftLeft.setPower(1);
            //robot.vLiftRight.setPower(1);
            sleep(2000);
            robot.vLiftRight.setPower(0);
            telemetry.addData("a:" , a);
            telemetry.update();

            //driveStrafe(0.3,'l',50000);
            robot.servo.setPosition(.25);
            sleep(2000);
            /* this code raises the lift to the right height, and turns turret, it is ready to score if we want to do it,
             robot.vLiftLeft.setTargetPosition(3100);
            robot.vLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vLiftLeft.setPower(-1);
            robot.vLiftRight.setTargetPosition(3100);
            robot.vLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vLiftRight.setPower(-1);
            sleep(2000);
            robot.turret.setTargetPosition(-450);
            robot.turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turret.setPower(0.5);
            sleep(2000);
            robot.hLift.setTargetPosition(225);
            robot.hLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hLift.setPower(0.3);
            sleep(1500);
             */
        }

    }
    public void driveStrafe(double speed, int d, double distance) { //speed always a pos num, char d is the direction either l or r
        // NOTE IMPORTANT: for some reason this method would not work unless i ran it at the very beginning of auto. otherwise, the motors would spin in incorrect directions. i have no clue why.

        double posS = 0; //position of the lateral encoder
        double posR = 0;
        double posL = 0;
        double lf = 0;
        double rf = 0;
        double lb = 0;
        double rb = 0;
        double mult = 1.0012;
        double mult2 = 1.000;

        lf = -speed * d;
        lb = speed * d;
        rf = speed * d;
        rb = -speed * d;

        runtime.reset();
        while(Math.abs(posS)<distance){//Math.abs(posS)<distance
            posS = encoderRear.getCurrentPosition();
            posR = encoderRight.getCurrentPosition();
            posL = encoderLeft.getCurrentPosition();
            robot.leftFront.setPower(lf);
            robot.leftBack.setPower(lb);
            robot.rightFront.setPower(rf);
            robot.rightBack.setPower(rb);
            telemetry.addData("EncoderRear:", posS);
            //telemetry.addData("Encoder Left:", posL);
            if(d == -1) {
                if (posR < posL) {
                   // rf = rf * mult2;
                    rb = rb * mult;
                    //lf = lf * mult;
                    lb = lb * mult;
                } else {
                    lf = lf * mult;
                    //lb = lb * mult2;
                    rf = rf * mult2; //mult2 is so that the robot doesn't become too tilted,
                    //rb = rb * mult;
                }
            }
            telemetry.update();
        }
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
    }

    public void driveStraight(double lf, double lb,
                           double rf, double rb, double distance) { //leftFront leftBack etc
        // sets power for all drive motors
        double posR = 0;
        double posL = 0;
        double mult = 1.0025;
        double mult2 = 1.0008;
        //encoderRight.setCurrentPosition() = 0;
        if(Math.abs(distance)<8000){
            mult = 1;
            mult2 = 1;
        }
        //robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(Math.abs((posR+posL)/2)<Math.abs(distance)){
            posR = encoderRight.getCurrentPosition();
            posL = encoderLeft.getCurrentPosition();
            robot.leftFront.setPower(lf);
            robot.leftBack.setPower(lb);
            robot.rightFront.setPower(rf);
            robot.rightBack.setPower(rb);
            telemetry.addData("EncoderRight:", posR);
            telemetry.addData("Encoder Left:", posL);
            if(posR < posL){

                rf = rf*mult;
                rb = rb*mult;
                lf = lf*mult2;
                lb = lb*mult2;
            }
            else{
                lf = lf*mult;
                lb = lb*mult;
                rf = rf*mult2; //mult2 is so that the robot doesn't become too tilted,
                rb = rb*mult2;
            }
            telemetry.update();
        }
        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);
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

