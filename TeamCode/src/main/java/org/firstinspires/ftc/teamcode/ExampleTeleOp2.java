package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ExampleTeleOp2 extends LinearOpMode {
    /**
     * Set power of a group of motors.
     *
     * @param power    power to set the motors to
     * @param dcMotors motors to set the powers
     */
    public void setPowers(double power, DcMotor... dcMotors) {
        for (DcMotor motor : dcMotors) {
            motor.setPower(power);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // This runs when the driver presses the INIT button.
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront"); // r
        DcMotor leftBack = hardwareMap.get(DcMotor.class, "leftBack"); // r
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            setPowers(1, leftFront, leftBack, rightFront, rightBack);
        }

    }
}
