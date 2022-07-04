package org.firstinspires.ftc.teamcode.old;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous(name="GearUp: Drive Autonomous 2" , group="GearUp")
//@Disabled
public abstract class Auto2 extends LinearOpMode {
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

    String alliance = "";
    String position = "";
    int startPosition = 0 ;
    
    // Positions
    int liftZero = 0;
    int liftBot = 220;
    int liftMid = 311;
    int liftTop = 500;

    // IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    
    //Vuforia
    private static final CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = true;
    private static final String VUFORIA_KEY =
                        "AU8mlRD/////AAABmQf7pySb9UJTkn6s0RIQ/wpYmsP0wPEI5Bdn5ggoVNaSvsTZh7oGyq2Z88EtLMqQHjBWi8Ycd05lqSM5GHY2TKUv2RTSdwUnGMr0ULOikKeG9w52H+tJfvT9WoAaqREMzcMuRzDpOAh+oJUXsHomA+7lbwsZnznqhrWh+k684Y1slhLRtmkPjeHw3x2pqyMwiiJehalWUdXlPzs2+fvMlOsAsbZoVGHOHkSKQ+4n6GaEB9b1vGRT69FFy1ePlXxJGODTwtkHdGi7hK/OJCcmhlRuOdcgGPxCaXrpAQVrV2pnnGBTRKRivd39Zt2JnyLkX1lKx6bcS4Y4c4uWhGBdTSELMzF6UdHknRjGoAvH2Kzc";
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model_20220114_173237.tflite";
    private static final String[] LABELS = {
        "markerForward"
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

        /** Wait for the game to begin */
        waitForStart();
        
//----------------------------------------------------------------------------------------------------------------
        
        if (position.equals("blue_barrier")) {
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
            int positionDuck = 1;

            if (duckPos < 0) { // duck is far right (looking away from the wall)
                telemetry.addData("RIGHT", "!");
                positionDuck = 3;
                telemetry.update();
                robot.gripperArm.setTargetPosition(-825);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.3);
                sleep(1000);
                sideDrive(0.2, 2);
                Straighty(0.5, 0, 10);
                robot.gripper.setPosition(0.8);    // close
                sleep(1500);
                robot.gripperArm.setTargetPosition(0);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.15);
                //sleep(30000);
                Straighty(0.5, 0, 37);
                rotate(-0.5, -82.5);
                sleep(500);
                rotate(-0.1, -90);
                Straighty(0.1, -90, 3);
            } else if (duckPos > 200) { // duck is middle
                telemetry.addData("MID", "!");
                telemetry.update();
                positionDuck = 2;
                robot.gripperArm.setTargetPosition(-825);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.3);
                sleep(1000);
                sideDrive(-0.1, 6.5);
                Straighty(0.5, 0, 10.2);
                robot.gripper.setPosition(0.8);    // close
                sleep(1500);
                robot.gripperArm.setTargetPosition(0);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.15);
                sideDrive(0.2, 7);
                Straighty(0.5, 0, 38);
                rotate(-0.5, -82.5);
                sleep(500);
                rotate(-0.1, -90);
                Straighty(0.1, -90, 2);
            } else { // duck is far left
                telemetry.addData("LEFT", "!");
                positionDuck = 1;
                telemetry.update();
                robot.gripperArm.setTargetPosition(-825);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.3);
                sleep(1000);
                Straighty(0.3, 0, 8);
                rotate(-0.2, 40);
                Straighty(0.1, 40, 3);
                robot.gripper.setPosition(0.8);    // close
                sleep(1500);
                robot.gripperArm.setTargetPosition(0);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.15);
                rotate(-0.5, 5);
                sleep(500);
                rotate(-0.1, 0);
                Straighty(0.5, 0, 40);
                rotate(-0.5, -82.5);
                sleep(500);
                rotate(-0.1, -90);
                Straighty(0.3, -90, 3);
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
            
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.5);
            //robot.lift.setPower(0.5);
            sleep(2000);
            robot.hopper.setPosition(0.75);
            sleep(2000);
            robot.hopper.setPosition(0.1);
            sleep(500);
            robot.lift.setTargetPosition(liftZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftSensor.getState()){
                robot.lift.setPower(0.25);
            }
            
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setPower(0);
            sleep(500);
            
            sideDrive(0.3, 28); 
            driveVert(-0.4, 68); // UNCOMMENT AT COMPETIION!
        }
        
        else if (position.equals("blue_carousel")) {
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

            if (duckPos < 0) { // duck is far right (looking away from the wall)
                telemetry.addData("RIGHT", "!");
                positionDuck = 3;
                telemetry.update();
                robot.gripperArm.setTargetPosition(-825);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.3);
                sleep(1000);
                sideDrive(0.2, 2);
                Straighty(0.5, 0, 10);
                robot.gripper.setPosition(0.8);    // close
                sleep(1500);
                robot.gripperArm.setTargetPosition(0);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.15);
                Straighty(0.5, 0, 40);
                rotate(-0.5, -90);
                Straighty(0.2, -90, 5);
                
            } else if (duckPos > 200) { // duck is middle
                telemetry.addData("MID", "!");
                telemetry.update();
                positionDuck = 2;
                robot.gripperArm.setTargetPosition(-825);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.3);
                sleep(1000);
                sideDrive(-0.2, 5);
                Straighty(0.5, 0, 9);
                robot.gripper.setPosition(0.8);    // close
                sleep(1500);
                robot.gripperArm.setTargetPosition(0);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.15);
                Straighty(0.5, 0, 40);
                rotate(-0.5, -90);
                Straighty(0.2, -90, 12);
            } else { // duck is far left
                telemetry.addData("LEFT", "!");
                positionDuck = 1;
                
                telemetry.update();
                robot.gripperArm.setTargetPosition(-825);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.3);
                sleep(1000);
                sideDrive(-0.1, 20);
                Straighty(0.5, 0, 10);
                robot.gripper.setPosition(0.8);    // close
                sleep(1500);
                robot.gripperArm.setTargetPosition(0);    //-775
                robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.gripperArm.setPower(0.15);
                Straighty(0.5, 0, 40);
                //rotate(-0.5, -82.5);
                //sleep(500);
                rotate(-0.5, -90);
                Straighty(0.2, -90, 24);
                //Straighty(0.1, 90, 2);
            }
            telemetry.update();
            /*sleep(500);
            driveVert(0.3,21.5);
            sleep(600);
            sideDrive(0.5, 27);*/
            
            if (positionDuck == 1){
                robot.lift.setTargetPosition(liftBot);
            } else if(positionDuck == 2){
                robot.lift.setTargetPosition(liftMid);
            } else if(positionDuck == 3){
                robot.lift.setTargetPosition(liftTop);
            }
            
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(0.5);
            sleep(1500);
            robot.hopper.setPosition(0.75);
            sleep(1500);
            robot.hopper.setPosition(0.1);
            sleep(500);
            robot.lift.setTargetPosition(liftZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (robot.liftSensor.getState()){
                robot.lift.setPower(0.25);
                /*
                if (robot.lift.getCurrentPosition() > 100)
                    robot.lift.setPower(-0.2);
                else
                    robot.lift.setPower(-0.1);
                */
            }
            
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lift.setPower(0);
            sleep(500);
            
          
            
            rotate(-0.4, -45);
            //rotate(-0.1, -40);
            //robot.intake.setPower(0.45);
            Straighty(-0.5, -40, 55);
            rotate(-0.2, 0);
            // robot.intake.setPower(0);
            Straighty(-0.2, 0, 12);
            robot.duck.setPower(-0.10);
            sleep(3000);
            robot.duck.setPower(0);
            rotate(-0.4, 0);
            Straighty(0.5, 0, 30);
        }
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
            front = Speed + (error / 50);
            back = Speed - (error / 50);
    
            average = (Math.abs(robot.leftFront.getCurrentPosition())+
                    Math.abs(robot.leftBack.getCurrentPosition())+
                    Math.abs(robot.rightFront.getCurrentPosition())+
                    Math.abs(robot.rightBack.getCurrentPosition()))/4;
    
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
                    Math.abs(robot.rightBack.getCurrentPosition()))/4;
    
            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/


        }

        driveMotor(0, 0, 0, 0);
    }
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    public void Straighty (double Speed, double heading, double targetInches){
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
                    robot.rightBack.getCurrentPosition())/4;

            /*telemetry.addData("heading",angles.firstAngle);
            telemetry.update();*/
        }
        driveMotor(0, 0, 0, 0);
    }
    
    //--------------------
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
                    robot.rightBack.getCurrentPosition())/4;

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
        tfodParameters.minResultConfidence = 0.7f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        // tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.loadModelFromFile(TFOD_MODEL_ASSET, LABELS);
    }
}
