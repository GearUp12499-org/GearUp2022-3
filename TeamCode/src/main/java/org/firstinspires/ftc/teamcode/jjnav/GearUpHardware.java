package org.firstinspires.ftc.teamcode.jjnav;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;


public class GearUpHardware {
    public HardwareMap hwMap =  null;

    //initializing motors,sensors, and servos
    public DcMotor leftFront;
    public DcMotor leftBack;
    public DcMotor rightFront;
    public DcMotor rightBack;
    public DcMotor vLiftLeft;
    public DcMotor vLiftRight;
    public static Encoder encoderLeft;
    public static Encoder encoderRight;
    public static Encoder encoderRear;
    public Servo servo = null;


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;
        try {
            leftFront = hwMap.get(DcMotorEx.class, "leftFront");
            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (IllegalArgumentException ignore) {
        }

        try {
            rightFront = hwMap.get(DcMotorEx.class, "rightFront");
            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException ignore) {
        }

        try {
            leftBack = hwMap.get(DcMotorEx.class, "leftBack");
            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        } catch (IllegalArgumentException ignore) {
        }
        try {
            rightBack = hwMap.get(DcMotorEx.class, "rightBack");
            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
            rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (IllegalArgumentException ignore) {
        }
        vLiftLeft= hwMap.get(DcMotorEx.class, "lift1");
        vLiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        vLiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLiftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vLiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vLiftRight= hwMap.get(DcMotorEx.class, "lift2");
        vLiftRight.setDirection(DcMotorSimple.Direction.FORWARD);
        vLiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vLiftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        vLiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servo = hwMap.get(Servo.class, "servo");
        servo.setPosition(0.1);


        encoderRight = new Encoder((DcMotorEx) rightFront);   // 0
        encoderLeft = new Encoder((DcMotorEx) leftFront);   // 1
        encoderRear = new Encoder((DcMotorEx)rightBack);    // 2


    }

}
