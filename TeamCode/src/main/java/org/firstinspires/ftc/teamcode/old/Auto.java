package org.firstinspires.ftc.teamcode.old;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="GearUp: Drive Autonomous" , group="GearUp")
//@Disabled
public abstract class Auto extends LinearOpMode {
    /* Declare OpMode members. */
    GearUpHardware          robot   = new GearUpHardware();   // Use GearUp hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV    = 537;    // eg: Neverest 40
    static final double DRIVE_GEAR_REDUCTION    = 0.707 ;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1416);
   
    int             target = 0;
    static final double     DRIVE_SPEED             = 0.7;
    static final double     TURN_SPEED              = 0.5;
    //float gains = 4;
    String alliance = "";
    String position = "";
    int startPosition = 0 ;
    
    // Positions
    int liftZero = 0;
    int liftBot = 900;
    int liftMid = 1100;
    int liftTop = 1625;
    int gripArmPos=0;
    double camPos=.175;
    double hopperUp = 0.37; // was .15 before
    double hopperDown = 1.0;


    // IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    
    //Vuforia
    private static final CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;
    private static final String VUFORIA_KEY =
                        "AU8mlRD/////AAABmQf7pySb9UJTkn6s0RIQ/wpYmsP0wPEI5Bdn5ggoVNaSvsTZh7oGyq2Z88EtLMqQHjBWi8Ycd05lqSM5GHY2TKUv2RTSdwUnGMr0ULOikKeG9w52H+tJfvT9WoAaqREMzcMuRzDpOAh+oJUXsHomA+7lbwsZnznqhrWh+k684Y1slhLRtmkPjeHw3x2pqyMwiiJehalWUdXlPzs2+fvMlOsAsbZoVGHOHkSKQ+4n6GaEB9b1vGRT69FFy1ePlXxJGODTwtkHdGi7hK/OJCcmhlRuOdcgGPxCaXrpAQVrV2pnnGBTRKRivd39Zt2JnyLkX1lKx6bcS4Y4c4uWhGBdTSELMzF6UdHknRjGoAvH2Kzc";
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model_20220326_015242.tflite";
    private static final String[] LABELS = {
        "marker"//markerBetter
    };
    
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    // Class Members
    private OpenGLMatrix lastLocation = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         * Set up Vuforia
         */
        robot.init(hardwareMap);
        
        // set up dc motors
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        robot.gripper.setPosition(0.1);
        robot.hopper.setPosition(hopperUp);
        
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update();

        // IMU setup
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gParameters.loggingEnabled      = true;
        gParameters.loggingTag          = "IMU";
        gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. IMU attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.2, 12.0/9.0);
        }
        
        robot.cameraAngle.setPosition(camPos);


        /** Wait for the game to begin */
        waitForStart();
        
//----------------------------------------------------------------------------------------------------------------
        if (position.equals("red_carousel")) {
            float duckPos = -10;
            int t = 0;
            for (; t < 20000 && opModeIsActive(); ++t) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            float cur = recognition.getLeft();
                            if (duckPos < 0)
                                duckPos = cur;
                            else if (duckPos > cur)
                                duckPos = cur;
                            
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                        if (duckPos < 0) {
                            break;
                        }
                    }
                }
            }
            
            telemetry.addData("t: ", t);
            telemetry.update();
            sleep(500);
            int positionDuck = 0;

            if (duckPos < 0) { // duck is far right (looking away from the wall)
                telemetry.addData("LEFT", "!");
                telemetry.update();
                positionDuck = 1;
                
                
            } else if (duckPos > 200) { // duck is middle
                telemetry.addData("RIGHT", "!");
                telemetry.update();
                positionDuck = 3;
                
               
            } else { // duck is far left
                telemetry.addData("MID", "!");
                positionDuck = 2;
                telemetry.update();
                //Straighty(0.1, 90, 2);
            }
            
            
            telemetry.update();
            
            // DUCKPOSITION WILL BE 3 EVERY TIME
            // int positionDuck = 3;
            
            if (positionDuck == 1){
                robot.lift.setTargetPosition(liftBot);
            } else if(positionDuck == 2){
                robot.lift.setTargetPosition(liftMid);
            } else if(positionDuck == 3){
                robot.lift.setTargetPosition(liftTop);
            }
            
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            
            sleep(750);
            robot.hopper.setPosition(hopperDown);
            sleep(1000);//was 2000
            robot.hopper.setPosition(hopperUp);
            sleep(250);
            robot.lift.setTargetPosition(liftZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftSensor.getState()){
                robot.lift.setPower(0.75);
            }
            
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setPower(0);
            
            int heading = 30;
            rotate(-0.3, heading);
            Straighty(0.3, heading, 7);
            
            sideIMU(-0.3, heading, 45);
            //sideIMU(-0.1, heading, 8);
            
            robot.duck.setPower(-0.10);
            sleep(4500);
            robot.duck.setPower(0);
            
            Straighty(0.3, heading, 31);
            sideIMU(-0.1, heading, 5);

            /*
            if (positionDuck == 3) {
                Straighty(0.5, 0, 6);
            }
            rotate(-0.3, 30);
            if(positionDuck == 3){
            sideIMU(.3, 30, -4);
            }
            */
            
            /*
            robot.gripperArm.setTargetPosition(-850);    // -825 before team marker change //-775 
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(0.3);
            sleep(750);
        
            robot.gripperArm.setTargetPosition(-850);    // -825 before team marker change //-775 
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(0.3);
            sleep(750);
            
            Straighty(0.3, 0, 1);
            sideIMU(.3, 0, -15);
            */
        } 
//----------------------------------------------------------------------------------------------------------------
        if (position.equals("blue_carousel")) {
            float duckPos = -10;
            int t = 0;
            for (; t < 20000 && opModeIsActive(); ++t) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            float cur = recognition.getLeft();
                            if (duckPos < 0)
                                duckPos = cur;
                            else if (duckPos > cur)
                                duckPos = cur;
                            
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                        if (duckPos < 0) {
                            break;
                        }
                    }
                }
            }
            
            telemetry.addData("t: ", t);
            telemetry.update();
            sleep(500);
            int positionDuck = 0;

            if (duckPos < 0) { // duck is far right (looking away from the wall)
                telemetry.addData("RIGHT", "!");
                telemetry.update();
                positionDuck = 3;
                
                
            } else if (duckPos > 200) { // duck is middle
                telemetry.addData("MID", "!");
                telemetry.update();
                positionDuck = 2;
                
               
            } else { // duck is far left
                telemetry.addData("LEFT", "!");
                positionDuck = 1;
                telemetry.update();
                //Straighty(0.1, 90, 2);
            }
            
            
            telemetry.update();
            
            // DUCKPOSITION WILL BE 3 EVERY TIME
            // int positionDuck = 3;
            
            if (positionDuck == 1){
                robot.lift.setTargetPosition(liftBot);
            } else if(positionDuck == 2){
                robot.lift.setTargetPosition(liftMid);
            } else if(positionDuck == 3){
                robot.lift.setTargetPosition(liftTop);
            }
            
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            
            sleep(750);
            robot.hopper.setPosition(hopperDown);
            sleep(1000);//was 2000
            robot.hopper.setPosition(hopperUp);
            sleep(250);
            robot.lift.setTargetPosition(liftZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftSensor.getState()){
                robot.lift.setPower(0.75);
            }
            
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setPower(0);
            
            int heading = 60;
            rotate(-0.3, heading);
            sideIMU(0.3, heading, 7);
            Straighty(-0.3, heading, 28);
            Straighty(-0.1, heading, 5);
            
            //sideIMU(-0.1, heading, 8);
            
            robot.duck.setPower(0.10);
            sleep(4500);
            robot.duck.setPower(0);
            
            sideIMU(0.4, heading, 42);
            Straighty(-0.3, heading, 14);
        } 
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        else if (position.equals("red_barrier")) {
            long start = System.currentTimeMillis();
            robot.duck.setDirection(DcMotor.Direction.FORWARD); 
            float duckPos = -10;
            int t = 0;
            for (; t < 5000 && opModeIsActive(); ++t) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            float cur = recognition.getLeft();
                            if (duckPos < 0)
                                duckPos = cur;
                            else if (duckPos > cur)
                                duckPos = cur;
                            
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                        if (duckPos < 0) {
                            break;
                        }
                    }
                }
            }
            
            telemetry.addData("t: ", t);
            telemetry.update();
            sleep(500);
            int positionDuck = 0;
            
            //duckPos = -10;

            if (duckPos < 0) { // duck is far right (looking away from the wall)
                telemetry.addData("LEFT", "!");
                telemetry.update();
                positionDuck = 1;
                
            } else if (duckPos <250) { // duck is middle
                telemetry.addData("MID", "!");
                telemetry.update();
                positionDuck = 2;
               
            } else { // duck is far left
                telemetry.addData("RIGHT", "!");
                positionDuck = 3;
                telemetry.update();
                //Straighty(0.1, 90, 2);
            }

            telemetry.update();
            
            if (positionDuck == 1){
                robot.lift.setTargetPosition(liftBot);
            } else if(positionDuck == 2){
                robot.lift.setTargetPosition(liftMid);
            } else if(positionDuck == 3){
                robot.lift.setTargetPosition(liftTop);
            }
            
            double heading = 60;
            
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1.0);
            robot.hopper.setPosition(0.65);
            
            sleep(750);
            robot.hopper.setPosition(hopperDown);
            sleep(600);//was 2000
            
            robot.hopper.setPosition(hopperUp);
            // sleep(250);
         
            robot.lift.setTargetPosition(liftZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (robot.liftSensor.getState()){
                robot.lift.setPower(1);//was 0.5
            // }
            
            // robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // robot.lift.setPower(0);
            
            // Go to warehouse & grab block
            rotate(-.4, heading);
            sideIMU(-1.0, heading, 12);
            Straighty(-1.0, heading, 40);
            // sideIMU(1.0, 90, -2);
            
            for (t = 0; t < 3; ++t) {
                get_block(-0.2, heading); // was 0.3
                            
                // rotate(-0.2, 90);
                sideIMU(-1.0, heading, 8);
                // sleep(1000);
                //Straighty(1.0, 70, 20);
                //front_wall(-0.3, 90, 40);
                floor(0.4, heading);
                robot.intake.setPower(1);
                Straighty(1.0, heading, 20); //was 30
                
                robot.lift.setTargetPosition(liftTop);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
                rotate(-.25, 0);//was 0.6
                
                robot.intake.setPower(0);
                
                robot.hopper.setPosition(0.65);
                
                sleep(750);
                robot.hopper.setPosition(hopperDown);
                sleep(600);//was 2000
                robot.hopper.setPosition(hopperUp);
                // sleep(250);
                robot.lift.setTargetPosition(liftZero);//liftZero
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // while (robot.liftSensor.getState()){
                    robot.lift.setPower(1);
                // }
                
                // robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // robot.lift.setPower(0);
                
                // Go to warehouse & grab block
                rotate(-.35, heading);
                sideIMU(-1.0, heading, 12);
                 
                Straighty(-1.0, heading, 40);
                // sideIMU(1.0, 90, -2);
                
                
                long elapsed = System.currentTimeMillis() - start;
                if (elapsed > 23000) {
                    break;
                }
            }
            
        }
        
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        
        else if (position.equals("blue_barrier")) {
            long start = System.currentTimeMillis();
            robot.duck.setDirection(DcMotor.Direction.FORWARD); 
            float duckPos = -10;
            int t = 0;
            for (; t < 5000 && opModeIsActive(); ++t) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            float cur = recognition.getLeft();
                            if (duckPos < 0)
                                duckPos = cur;
                            else if (duckPos > cur)
                                duckPos = cur;
                            
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                        if (duckPos < 0) {
                            break;
                        }
                    }
                }
            }
            
            telemetry.addData("t: ", t);
            telemetry.update();
            sleep(500);
            int positionDuck = 0;
            
            //duckPos = -10;

            if (duckPos < 0) { // duck is far right (looking away from the wall)
                telemetry.addData("LEFT", "!");
                telemetry.update();
                positionDuck = 1;
                
            } else if (duckPos <250) { // duck is middle
                telemetry.addData("MID", "!");
                telemetry.update();
                positionDuck = 2;
               
            } else { // duck is far left
                telemetry.addData("RIGHT", "!");
                positionDuck = 3;
                telemetry.update();
                //Straighty(0.1, 90, 2);
            }

            telemetry.update();
            
            if (positionDuck == 1){
                robot.lift.setTargetPosition(liftBot);
            } else if(positionDuck == 2){
                robot.lift.setTargetPosition(liftMid);
            } else if(positionDuck == 3){
                robot.lift.setTargetPosition(liftTop);
            }
            
            double heading = 310;
            
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1.0);
            robot.hopper.setPosition(0.65);
            
            sleep(750);
            robot.hopper.setPosition(hopperDown);
            sleep(750);//was 2000
            
            robot.hopper.setPosition(hopperUp);
            // sleep(250);                                                                                              
         
            robot.lift.setTargetPosition(liftZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // while (robot.liftSensor.getState()){
                robot.lift.setPower(1);//was 0.5
            // }
            
            // robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // robot.lift.setPower(0);
            
            // Go to warehouse & grab block
            rotate(-.4, heading);
            sideIMU(1.0, heading, 12);
            Straighty(-1.0, heading, 40);
            // sideIMU(1.0, 90, -2);
            
            for (t = 0; t < 3; ++t) {
                get_block(-0.2, heading); // was 0.3
                            
                // rotate(-0.2, 90);
                sideIMU(1.0, heading, 8);
                // sleep(1000);
                //Straighty(1.0, 70, 20);
                //front_wall(-0.3, 90, 40);
                floor(0.4, heading-10);
                robot.intake.setPower(1);
                Straighty(1.0, heading, 20);
                
                sleep(500);
                robot.lift.setTargetPosition(liftTop);
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.lift.setPower(1.0);
                rotate(-.25, 355);//was 0.6
                
                robot.intake.setPower(0);
                
                robot.hopper.setPosition(0.65);
                
                sleep(750);
                robot.hopper.setPosition(hopperDown);
                sleep(600);//was 2000
                robot.hopper.setPosition(hopperUp);
                // sleep(250);
                robot.lift.setTargetPosition(liftZero);//liftZero
                robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                // while (robot.liftSensor.getState()){
                    robot.lift.setPower(1);
                // }
                
                // robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                // robot.lift.setPower(0);
                
                // Go to warehouse & grab block
                rotate(-.35, heading);
                sideIMU(1.0, heading, 12);
                 
                Straighty(-1.0, heading, 40);
                // sideIMU(1.0, 90, -2);
                
                
                long elapsed = System.currentTimeMillis() - start;
                if (elapsed > 23000) {
                    break;
                }
            }
        }
    }
//------------------------------------------------------------------------------------------------------------------
    public void front_wall(double Speed, double heading, double position) {
    /* uses IMU to remain on absolute heading with proportional course correction
    Speed - speed of wheels
    Heading - desired absolute direction
    */
        double error;
        double right = Speed;
        double left = Speed;
        DistanceSensor sensorRange4 = hardwareMap.get(DistanceSensor.class,"wallDistance"); 
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange4;
         
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("range_back", String.format("%.01f in", sensorRange4.getDistance(DistanceUnit.INCH)));
         telemetry.update();
        while (Math.abs(sensorRange4.getDistance(DistanceUnit.INCH) - position) > 1) {
            if (sensorRange4.getDistance(DistanceUnit.INCH) < position) {
                driveMotor(-left, -left, -right, -right);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                error = heading - angles.firstAngle;
                if (error > 180)
                    error -=360;
                if (error <= -180)
                    error += 360;
                left = Speed + (error / 50);
                right = Speed - (error / 50);
            }
            else if (sensorRange4.getDistance(DistanceUnit.INCH)> position) {
                driveMotor(left, left, right, right);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                error = heading - angles.firstAngle;
                if (error > 180)
                    error -=360;
                if (error <= -180)
                    error += 360;
                left = Speed - (error / 50);
                right = Speed + (error / 50);
            }
        }
        driveMotor(0, 0, 0, 0);
    }
//------------------------------------------------------------------------------------------------------------------
    public void floor(double Speed, double heading) {
    /* uses IMU to remain on absolute heading with proportional course correction
    Speed - speed of wheels
    Heading - desired absolute direction
    */
        
        if (robot.floorColor instanceof SwitchableLight) {
            ((SwitchableLight)robot.floorColor).enableLight(true);
        }
        
        NormalizedRGBA colors = robot.floorColor.getNormalizedColors();

        // Color.colorToHSV(colors.toColor(), hsvValues);
        double error;
        double right = Speed;
        double left = Speed;
         
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        while (colors.alpha < 0.9 && runtime.seconds() < 4) {
            colors = robot.floorColor.getNormalizedColors();
            driveMotor(left, left, right, right);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = heading - angles.firstAngle;
            if (error > 180)
                error -=360;
            if (error <= -180)
                error += 360;
            left = Speed + (error / 50);
            right = Speed - (error / 50);

            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/
        }
        driveMotor(0, 0, 0, 0);
        
        
        telemetry.addData("I AM DONE PASSING THE WHITE LINE", "!!!!!!");
        telemetry.update();
    }

//------------------------------------------------------------------------------------------------------------------
    public void sideIMU (double Speed, double heading, double targetInches){
    /* uses IMU to remain on absolute heading with proportional course correction
    Speed - speed of wheels
    Heading - desired absolute direction
    targetInches - desired distance in inches
    */
        double error;
        double front = Speed;
        double back = Speed;
        double average = 0;
        double targetCounts = targetInches * COUNTS_PER_INCH;
    
        //motor setup
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetInches < 0){
            Speed *= -1;
        }
        while (Math.abs(targetCounts) >= Math.abs(average)){
            driveMotor(front, -back, -front, back);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle - heading;
            if (error > 180)
                error -=360;
            if (error <= -180)
                error += 360;
            front = Speed - (error / 50);
            back = Speed + (error / 50);
    
            average = (Math.abs(robot.leftFront.getCurrentPosition())+
                    Math.abs(robot.leftBack.getCurrentPosition())+
                    Math.abs(robot.rightFront.getCurrentPosition())+
                    Math.abs(robot.rightBack.getCurrentPosition()))/4f;
    
            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/
                                //telemetry.addData("Error", error);
                                                   // sleep(4000);


        }

        driveMotor(0, 0, 0, 0);
    }
    //--------------------------
     public void sideDrive (double Speed, double targetInches){
    /* uses IMU to remain on absolute heading with proportional course correction
    Speed - speed of wheels
    Heading - desired absolute direction
    targetInches - desired distance in inches
    */
       // double error;
        double front = Speed;
        double back = Speed;
        double average = 0;
        double targetCounts = targetInches * COUNTS_PER_INCH;
    
        //motor setup
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetInches < 0){
            Speed *= -1;
        }
        while (Math.abs(targetCounts) >= Math.abs(average)){
            driveMotor(front, -back, -front, back);
            /*angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = angles.firstAngle - heading;
            if (error > 180)
                error -=360;
            if (error <= -180)
                error += 360;
            front = Speed + (error / 50);
            back = Speed - (error / 50);
            */
            average = (Math.abs(robot.leftFront.getCurrentPosition())+
                    Math.abs(robot.leftBack.getCurrentPosition())+
                    Math.abs(robot.rightFront.getCurrentPosition())+
                    Math.abs(robot.rightBack.getCurrentPosition()))/4f;
    
            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/


        }

        driveMotor(0, 0, 0, 0);
    }
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    public void Straighty(double Speed, double heading, double targetInches){
        /* uses IMU to remain on absolute heading with proportional course correction
        Speed - speed of wheels
        Heading - desired absolute direction
        targetInches - desired distance in inches
        */
        double error;
        double right = Speed;
        double left = Speed;
        double average = 0;
        double targetCounts = targetInches * COUNTS_PER_INCH;

        //motor setup
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetInches < 0){
            Speed *= -1;
        }
        while (Math.abs(targetCounts) >= Math.abs(average)){
            driveMotor(left, left, right, right);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = heading - angles.firstAngle;
            if (error > 180)
                error -=360;
            if (error <= -180)
                error += 360;
            left = Speed + (error / 50);
            right = Speed - (error / 50);

            average = (robot.leftFront.getCurrentPosition()+
                    robot.leftBack.getCurrentPosition()+
                    robot.rightFront.getCurrentPosition()+
                    robot.rightBack.getCurrentPosition())/4f;

            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/
        }
        driveMotor(0, 0, 0, 0);
    }
    //---------------------------------------------------------------------------------------------------
    public void get_block(double Speed, double heading){
        /* uses IMU to remain on absolute heading with proportional course correction
        Speed - speed of wheels
        Heading - desired absolute direction
        targetInches - desired distance in inches
        */
        double error;
        double right = Speed;
        double left = Speed;
        double average = 0;

        //motor setup
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        if (robot.ramp instanceof SwitchableLight) {
            ((SwitchableLight)robot.ramp).enableLight(true);
        }
        if (robot.hopperColor instanceof SwitchableLight) {
            ((SwitchableLight)robot.hopperColor).enableLight(true);
        }
        
                
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = robot.hopperColor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        
        robot.intake.setPower(0.2); // 0.4 when low voltage!
        driveMotor(0, 0, 0, 0);
        
        runtime.reset();
        boolean finish = false;
        while (robot.ramp instanceof DistanceSensor &&
            ((DistanceSensor) robot.ramp).getDistance(DistanceUnit.CM) >= 5 &&
            robot.hopperColor instanceof DistanceSensor &&
            ((DistanceSensor) robot.hopperColor).getDistance(DistanceUnit.CM) >= 7) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) robot.ramp).getDistance(DistanceUnit.CM));
            telemetry.update();
    
            colors = robot.hopperColor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            
            driveMotor(left, left, right, right);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            error = heading - angles.firstAngle;
            if (error > 180)
                error -=360;
            if (error <= -180)
                error += 360;
            left = Speed + (error / 50);
            right = Speed - (error / 50);

            average = (robot.leftFront.getCurrentPosition()+
                    robot.leftBack.getCurrentPosition()+
                    robot.rightFront.getCurrentPosition()+
                    robot.rightBack.getCurrentPosition())/4f;

            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/
            
            if (runtime.seconds() > 5) {
                finish = true;
                break;
            }
        }
        
        driveMotor(0, 0, 0, 0);
        if (robot.hopperColor instanceof DistanceSensor &&
            ((DistanceSensor) robot.hopperColor).getDistance(DistanceUnit.CM) >= 6.5) {
            robot.intake.setPower(0);
        } else {
            robot.intake.setPower(-0.5);
        }
        
        if (finish) {
            sleep(30000);
        }
    }
    //---------------------------------------------------------------------------------------------------
    public void driveVert(double Speed, double targetInches) {
        /* uses IMU to remain on absolute heading with proportional course correction
        Speed - speed of wheels
        Heading - desired absolute direction
        targetInches - desired distance in inches
        */
        double error;
        double right = Speed;
        double left = Speed;
        double average = 0;
        double targetCounts = targetInches * COUNTS_PER_INCH;

        //motor setup
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetInches < 0){
            Speed *= -1;
        }
        while (Math.abs(targetCounts) >= Math.abs(average)){
            driveMotor(left, left, right, right);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            average = (robot.leftFront.getCurrentPosition()+
                    robot.leftBack.getCurrentPosition()+
                    robot.rightFront.getCurrentPosition()+
                    robot.rightBack.getCurrentPosition())/4f;

            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/
        }
        driveMotor(0, 0, 0, 0);
    }
    
    //-------------------------------------
     public void driveDiagonal(double Speed, double targetInches) {
        /* uses IMU to remain on absolute heading with proportional course correction
        Speed - speed of wheels
        Heading - desired absolute direction
        targetInches - desired distance in inches
        */
        double error;
        double right = Speed;
        double left = Speed;
        double average = 0;
        double targetCounts = targetInches * COUNTS_PER_INCH;

        //motor setup
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (targetInches < 0){
            Speed *= -1;
        }
        while (Math.abs(targetCounts) >= Math.abs(average)){
            driveMotor(left, 0, 0, right);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            average = (robot.leftFront.getCurrentPosition()+
                    robot.leftBack.getCurrentPosition()+
                    robot.rightFront.getCurrentPosition()+
                    robot.rightBack.getCurrentPosition())/4f;

            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/
        }
        driveMotor(0, 0, 0, 0);
    }
    
//---------------------------------------------------------

//  Rotation with proportional correction - coded 12/26/2020
//  DEGREES ARE MEASURED COUNTERCLOCKWISE TO INITIAL POSITION!
//  Robot takes shortest path to get to the specified heading

    public void rotate(double speed, double heading) {
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive() && !onHeading(speed, heading));
    }
    public boolean onHeading(double speed, double angle) {
        double PCoeff=0.1, threshold=0.5; //PCoeff = proportional coefficient, threshold = error threshold
        boolean ret=false;
        
        double error=getError(angle);
        double leftSpeed, rightSpeed, steer;
        
        if (Math.abs(error)<=threshold) {
            ret=true;
            leftSpeed=rightSpeed=0.0;
        }
        else {
            steer=getSteer(error, PCoeff);
            rightSpeed=speed*steer;
            leftSpeed=-rightSpeed;
        }
        
        driveMotor(leftSpeed, leftSpeed, rightSpeed, rightSpeed);
        
        return ret;
    }
    public double getError(double target) {
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentError=target-angles.firstAngle;
        while (currentError>180)
            currentError-=360;
        while (currentError<=-180)
            currentError+=360;
        return currentError;
    }
    public double getSteer(double error, double PCoeff) { //clips (angula error)*0.1 to a number between -1 and 1
        return Range.clip(error*PCoeff, -1, 1);
    }
//------------------------------------------------------------------------------------
    public void driveMotor(double leftFront, double leftBack,
                           double rightFront, double rightBack){
    // sets power for all drive motors
        robot.leftFront.setPower(leftFront);
        robot.leftBack.setPower(leftBack);
        robot.rightFront.setPower(rightFront);
        robot.rightBack.setPower(rightBack);
    }
//--------------------------------
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam2");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.65f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
