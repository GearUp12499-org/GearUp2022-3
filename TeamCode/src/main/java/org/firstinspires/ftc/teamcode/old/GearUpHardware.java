package org.firstinspires.ftc.teamcode.old;
//import from qualcomm library

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class GearUpHardware {
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

    public DigitalChannel liftSensor = null;

    public Servo hopper = null;
    public Servo gripper = null;
    public Servo cameraAngle = null;
    public RevBlinkinLedDriver lights = null;

    public NormalizedColorSensor hopperColor = null;
    public NormalizedColorSensor ramp = null;
    public NormalizedColorSensor floorColor = null;
    //public NormalizedColorSensor jj = null;

    private float gain = 4;

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

        liftSensor = hwMap.get(DigitalChannel.class, "liftSensor");

        hopper = hwMap.get(Servo.class, "hopper");
        gripper = hwMap.get(Servo.class, "gripper");
        cameraAngle = hwMap.get(Servo.class, "cameraAngle");

        lights = hwMap.get(RevBlinkinLedDriver.class, "lights");

        ramp = hwMap.get(NormalizedColorSensor.class, "ramp");
        //jj = hwMap.get(NormalizedColorSensor.class, "jj");
        hopperColor = hwMap.get(NormalizedColorSensor.class, "hopperColor");
        floorColor = hwMap.get(NormalizedColorSensor.class, "floorColor");

        ramp.setGain(gain);
        hopperColor.setGain(gain);
        //jj.setGain(gain);
        floorColor.setGain(gain);

        //initialize direction of motors and servos
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        duck.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        gripperArm.setDirection(DcMotor.Direction.REVERSE);

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

        liftSensor.setMode(DigitalChannel.Mode.INPUT);
    }

}
