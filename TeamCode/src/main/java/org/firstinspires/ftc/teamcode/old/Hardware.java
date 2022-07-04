package org.firstinspires.ftc.teamcode.old;
//import from qualcomm library
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import java.lang.annotation.Target;
import java.lang.Object;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo; 
import com.qualcomm.robotcore.robot.Robot;



public class Hardware {
  public HardwareMap hwMap =  null;
  
  //initializing motors,sensors, and servos
  public DcMotor leftFront = null;
  public DcMotor leftBack = null;
  public DcMotor rightFront = null;
  public DcMotor rightBack = null;
  public DcMotor duck = null;
  public DcMotor lift = null;
  public DcMotor intake = null;
  public DcMotor gripperArm = null;
  
  public Servo hopper = null;
  public Servo gripper = null;
  
  /* Initialize standard Hardware interfaces */
  public void init(HardwareMap ahwMap) {
    
    // Save reference to Hardware map
    hwMap = ahwMap;
    
    // Define and Initialize Motors and servos and sensors
    leftFront  = hwMap.get(DcMotor.class, "leftFront");
    leftBack = hwMap.get(DcMotor.class, "leftBack");
    rightFront = hwMap.get(DcMotor.class, "rightFront");
    rightBack= hwMap.get(DcMotor.class, "rightBack");
    duck= hwMap.get(DcMotor.class, "duck");
    lift= hwMap.get(DcMotor.class, "lift");
    intake = hwMap.get(DcMotor.class, "intake");
    gripperArm = hwMap.get(DcMotor.class, "gripperArm");
     
    hopper = hwMap.get(Servo.class, "hopper");
    gripper = hwMap.get(Servo.class, "gripper");
    
    //initialize direction of motors and servos
    leftFront.setDirection(DcMotor.Direction.FORWARD); 
    leftBack.setDirection(DcMotor.Direction.FORWARD); 
    rightFront.setDirection(DcMotor.Direction.REVERSE);
    rightBack.setDirection(DcMotor.Direction.REVERSE);
    //duck.setDirection(DcMotor.Direction.FORWARD); 
    lift.setDirection(DcMotor.Direction.FORWARD);
    intake.setDirection(DcMotor.Direction.FORWARD);
    gripperArm.setDirection(DcMotor.Direction.REVERSE);
      NormalizedColorSensor ramp;

    // Set all motors to zero power
    leftFront.setPower(0);
    leftBack.setPower(0);
    rightFront.setPower(0);
    rightBack.setPower(0);
    duck.setPower(0);
    lift.setPower(0);
    intake.setPower(0);
    gripperArm.setPower(0);
    
    // RUN_USING_ENCODERS if encoders are installed.
    leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    duck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    gripperArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
  }
  
}
