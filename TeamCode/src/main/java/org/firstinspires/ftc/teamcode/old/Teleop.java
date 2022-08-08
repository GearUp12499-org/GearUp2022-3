package org.firstinspires.ftc.teamcode.old;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@TeleOp(name="GearUp: Drive TeleOp", group="Pushbot")
@Disabled
public abstract class Teleop extends LinearOpMode {
    GearUpHardware robot = new GearUpHardware();
    public ElapsedTime runtime = new ElapsedTime();
    // Drive
    double left = 0;
    double right = 0;
    double sideSpeed = 1;
    
    // Lift & gripper positions
    int liftZero = 0;
    /*
     540
        291
        128
        before it was 850 1250 2000
    */
    // 220 311 590
    int liftBot = 400;
    int liftMid = liftBot + 500;
    int liftTop = liftMid + 725; // was 580
    int gripArmPos=0;
    double cameraAngle=.175; //0.5 good for red barrier :) .175 for red caro

    String alliance = "";
    boolean gripperOpen = true;
    double gripPos;
    double hopperUp = 0.35; // was .15 before
    double hopperDown = 1.0;

    boolean objInHopper = false;
    boolean objInRamp = false;

    RevBlinkinLedDriver.BlinkinPattern basePattern;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    
    // IMU
    BNO055IMU imu;

    public void init_imu() {
        // IMU setup
        BNO055IMU.Parameters gParameters = new BNO055IMU.Parameters();
        gParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        gParameters.loggingEnabled      = true;
        gParameters.loggingTag          = "IMU";
        gParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gParameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        telemetry.addData("IMU", "Initialized");
        telemetry.update();
    }

    public void init_robot() {
        robot.init(hardwareMap);

        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.duck.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.gripperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        robot.cameraAngle.setPosition(cameraAngle);
        robot.hopper.setPosition(hopperUp);
        robot.gripper.setPosition(0.8);       // full open
        // robot.lift.setTargetPosition(0);

        
        robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.duck.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.gripperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.duck.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.gripperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        init_imu();
        
        if (alliance.equals("Red")) {
            robot.duck.setDirection(DcMotor.Direction.FORWARD);
            basePattern = RevBlinkinLedDriver.BlinkinPattern.RED;
            pattern = basePattern;
            
        } else {
            robot.duck.setDirection(DcMotor.Direction.REVERSE); 
            basePattern = RevBlinkinLedDriver.BlinkinPattern.BLUE;
            pattern = basePattern;
        }
        
        runtime.reset();
        while (runtime.seconds() < 3 && robot.liftSensor.getState()){
            robot.lift.setPower(-0.75);//-0.25
        }
        int delta = robot.lift.getCurrentPosition();
        
        liftZero += delta;
        liftBot += delta;
        liftMid += delta;
        liftTop += delta;
    }

    @Override
    public void runOpMode() {
        init_robot();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //telemetry.addData("Status", "Running");
            drive();
            lift();
            hopper();
            intake();
            gripperArm();
            gripper();
            ducky();
            lights();
            telemetry.update();
        }
    }

//===================================================================
    public void drive() {

        
        if (gamepad1.dpad_up) { // left trigger down is 1.0
            double speed = (0.25 + gamepad1.left_trigger * 0.75) * -1;
            driveMotor(speed, speed, speed, speed);
            
            /* replaced with a proportional speed increase instead of binary at left trigger of 0.5
            double speed;
            if(gamepad1.left_trigger>0.5){
                speed = -1;
                driveMotor(speed,speed,speed,speed);
            }
            else{
            speed=-1*(.25);
            driveMotor(speed, speed, speed, speed);
            }
            */
        }
        
        else if (gamepad1.dpad_down) {
            double speed = 0.25 + gamepad1.left_trigger * 0.75;
            driveMotor(speed, speed, speed, speed);
        }
        else if (gamepad1.dpad_left) {
            double front=sideSpeed*0.5+gamepad1.left_trigger*0.5, back=-front;
            driveMotor(front, back, back, front);
        }
        else if (gamepad1.dpad_right) {
            double back=sideSpeed*0.5+gamepad1.left_trigger*0.5, front=-back;
            driveMotor(front, back, back, front);
        }
        else {
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            driveMotor(left, left, right, right);
        }
        /*
        telemetry.addData("RB", robot.rightBack.getCurrentPosition());
        telemetry.addData("RF", robot.rightFront.getCurrentPosition());
        telemetry.addData("LB", robot.leftBack.getCurrentPosition());
        telemetry.addData("LF", robot.leftFront.getCurrentPosition());
        telemetry.update();
        */
        if (robot.ramp instanceof SwitchableLight) {
            ((SwitchableLight)robot.ramp).enableLight(true);
        }
        
        if (robot.ramp instanceof DistanceSensor) {
            double here = ((DistanceSensor) robot.ramp).getDistance(DistanceUnit.CM);
            telemetry.addData("Ramp distance (cm)", "%.3f", here);
            objInRamp = (here < 6);
        }

    }
//===================================================================
    public void driveMotor(double leftFront, double leftBack,
                           double rightFront, double rightBack){
    // sets power for all drive motors
        robot.leftFront.setPower(leftFront);
        robot.leftBack.setPower(leftBack);
        robot.rightFront.setPower(rightFront);
        robot.rightBack.setPower(rightBack);
    }
//===================================================================
    public void reset_imu() {
        if (gamepad1.b) {
            init_imu();
        }
    }
//===================================================================
    public void ducky() {
        if (gamepad1.x) {
            //robot.duck.setPower(0.65); remember to put this back you moron lmao
            robot.rightFront.setPower(1); 
        } else if(gamepad1.b){
            robot.duck.setPower(-0.65);
        } /*else if (objInHopper) {
            robot.duck.setPower(0.1);
        } */else{
            robot.duck.setPower(0);
        }
    }
//-----------------------------------
    public void lift(){
        int position = robot.lift.getCurrentPosition();
        
        /*
        telemetry.addData("lift", position);
        if(gamepad2.x){
            robot.lift.setPower(0.3);
            telemetry.addData("liftPosition", position);
            telemetry.update();

        }
       else if(gamepad2.b){
            robot.lift.setPower(-0.3);
            telemetry.addData("liftPosition", position);
            telemetry.update();
       }
        else{
            robot.lift.setPower(0);
            telemetry.addData("liftPosition", position);
                    telemetry.update();


        }
        */
        
        /*
        if (digitalTouch.getState()) {
            telemetry.addData("Digital Touch", "Is Not Pressed");
        } else {
            telemetry.addData("Digital Touch", "Is Pressed");
        }
        */
        
        telemetry.update();
        //power 0.5 with old motors
        if(gamepad2.a){ //reset
            robot.hopper.setPosition(hopperUp);
            // the hopper was closing simultaneously w the lift coming down
            // and hitting part of the robot, thus sleep may help prevent this
            //sleep(200); 
            robot.lift.setTargetPosition(liftZero);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(-0.75);
            /*runtime.reset();
            while (runtime.seconds() < 3 && robot.liftSensor.getState()){//robot.liftSensor.getState() runtime.seconds() < 3 &&
               while (position >0)
                robot.lift.setPower(-0.75);//-0.25
              
               
            }*/
         //   while(position<=500 && position > 80);
             //robot.lift.setPower(-0.1);
            //robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.lift.setPower(0);
        }
        else if(gamepad2.x){ // lowest        0.5
            robot.lift.setTargetPosition(liftBot);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
        }
        else if(gamepad2.y){ // middle
            robot.lift.setTargetPosition(liftMid);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
        }
        else if(gamepad2.b){ // highest
            robot.lift.setTargetPosition(liftTop);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setPower(1);
            robot.hopper.setPosition(0.65);
          
           

          
        }
            
        
        if (robot.hopperColor instanceof SwitchableLight) {
            ((SwitchableLight)robot.hopperColor).enableLight(true);
        }
        
        if (robot.hopperColor instanceof DistanceSensor) {
            double here = ((DistanceSensor) robot.hopperColor).getDistance(DistanceUnit.CM);
            telemetry.addData("Hopper distance (cm)", "%.3f", here);
            objInHopper = (here < 6.5);
            telemetry.update();
        }
        
    }
//===================================
    public void hopper(){
        if(gamepad2.right_bumper){
            robot.hopper.setPosition(hopperDown);
            // sleep(1000);
        }
        
        if (objInHopper) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        } else if (objInRamp) {
            // pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }else {
            pattern = basePattern;
        }
    }
//-----------------------------------
    public void intake(){
        int position = robot.lift.getCurrentPosition();

        if(gamepad1.left_bumper && position < 100){
            robot.intake.setPower(1);//0.75
        }
        else if(gamepad1.right_bumper){
            robot.intake.setPower(-0.75);
        }
        else{
            robot.intake.setPower(0);
        }
    }
//-----------------------------------
    public void gripper(){
        
        gripPos = robot.gripper.getPosition();
        if(gamepad2.left_bumper){
            /*
            if(gripPos>0.5){
                robot.gripper.setPosition(0.5);   // partial open
                sleep(200);
                gripperOpen = true;
            }
            */
            
            // gripper has been initialized to 0.1 at beginning
            if(gripPos==0.8){
                robot.gripper.setPosition(0.1);   // full open
                sleep(200);
                gripperOpen = true;
            }
            else{
                robot.gripper.setPosition(0.8);    // close
                sleep(200);
                gripperOpen = false;
            }
        }
    }
    
    public void gripperArm(){
        
        int gripperPosition = robot.gripperArm.getCurrentPosition();
        telemetry.addData("gripperPosition", 1.0 * gripperPosition);
        telemetry.addData("gripperTargetPosition", 1.0 * gripArmPos);
        
                          
        if(gamepad2.dpad_down){ // grab team shipping element
            robot.gripperArm.setTargetPosition(-850);    // -825 before team marker change //-775 
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(0.3);
            telemetry.addData("dpad", "down");
        }
        if(gamepad2.dpad_up){ // bring back to start
            robot.gripperArm.setTargetPosition(0);
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(0.15);
            telemetry.addData("dpad", "up");
        }
        
        if(gamepad2.dpad_left){ // vertical
            robot.gripperArm.setTargetPosition(-300);
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(0.2);
        }
        if(gamepad2.dpad_right){ // down by increments
            robot.gripperArm.setTargetPosition(gripperPosition - 15);
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(0.2);
        }
        
        if(gamepad2.left_stick_y>0.1 && gripperPosition>-850+10){ // down continuous
            int change = 40; // 10 + (int) (40*Math.abs(gamepad2.left_stick_y)*10);
            robot.gripperArm.setTargetPosition(gripperPosition - change); // negative change
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(0.3);
        }        
        if(gamepad2.left_stick_y<-0.1 && gripperPosition<0){ // up by increments
            int change = 40; //10 + (int) (40 * Math.abs(gamepad2.left_stick_y)*10);
            robot.gripperArm.setTargetPosition(gripperPosition + change); // positive change
            robot.gripperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.gripperArm.setPower(-0.3);
        }
        telemetry.addData("left stick", gamepad2.left_stick_y);
        
        telemetry.update();
        
     /*   if(robot.gripperArm.getCurrentPosition()>120 && robot.gripperArm.getTargetPosition()==250){
            robot.gripperArm.setPower(0);
            robot.gripperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if(robot.gripperArm.getCurrentPosition()<50 && robot.gripperArm.getTargetPosition()==0){
            robot.gripperArm.setPower(0);
            robot.gripperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); 
        }  */
        
    }
    public void lights(){
        robot.lights.setPattern(pattern);
    }
//-----------------------------------

}
