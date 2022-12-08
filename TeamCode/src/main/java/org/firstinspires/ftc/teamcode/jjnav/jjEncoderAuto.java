package org.firstinspires.ftc.teamcode.jjnav;

import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderRight;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import com.qualcomm.robotcore.hardware.Servo;

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

        import java.util.List;

        import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
        import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

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
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftFront.getCurrentPosition(),
                robot.rightFront.getCurrentPosition());
        telemetry.update();
        waitForStart();

//----------------------------------------------------------------------------------------------------------------
        if (position.equals("encoderDriveTest")){
            robot.servo.setPosition(1);
            sleep(2000);
            robot.vLiftLeft.setTargetPosition(800);
            robot.vLiftLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vLiftLeft.setPower(-1);
            robot.vLiftRight.setTargetPosition(800);
            robot.vLiftRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vLiftRight.setPower(-1);
            sleep(2000);
            //driveStraight(0.3,0.3,0.3,0.3, 90000); //commented out to test lift/gripper, but this is right distance from wall to center of second tile.
            //driveStrafe(0.3,'l',4000);
        }
    }
    public void driveStrafe(double speed, char d, double distance) { //speed always a pos num, char d is the direction either l or r
        // sets power for all drive motors
        double posR = 0;
        double posL = 0;
        double posS = 0; //position of the lateral encoder
        double mult = 1.0025;
        double mult2 = 1.0008;
        double lf = 0;
        double rf =0;
        double lb =0;
        double rb = 0;

        if(d == 'l'){
            lf = -speed;
            lb = speed;
            rf = speed;
            rb = -speed;
        }
        if(d == 'r'){
            lf = speed;
            lb = -speed;
            rf = -speed;
            rb = speed;
        }
        while((posS)<distance ){
            posS = encoderRear.getCurrentPosition();
            posR = encoderRight.getCurrentPosition();
            posL = encoderLeft.getCurrentPosition();
            robot.leftFront.setPower(lf);
            robot.leftBack.setPower(lb);
            robot.rightFront.setPower(rf);
            robot.rightBack.setPower(rb);
            telemetry.addData("EncoderRight:", posR);
            telemetry.addData("Encoder Left:", posL);
            if(posR < posL){
                rf = rf*mult2;
                rb = rb*mult;
                lf = lf*mult;
                lb = lb*mult2;
            }
            else{
                lf = lf*mult;
                lb = lb*mult2;
                rf = rf*mult2; //mult2 is so that the robot doesn't become too tilted,
                rb = rb*mult;
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
        while((posR+posL)/2<distance ){
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
}

