package org.firstinspires.ftc.teamcode.jjnav;

import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.jjnav.GearUpHardware.encoderRight;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
            driveMotor(0.5,0.5,0.5,0.5, 100);

        }
    }

    public void driveMotor(double leftFront, double leftBack,
                           double rightFront, double rightBack, double distance) {
        // sets power for all drive motors
        double posR = encoderRight.getCurrentPosition();
        double posL = encoderLeft.getCurrentPosition();
        double avg = (posR+posL)/2;
        runtime.reset();
        while(avg<distance ){
            robot.leftFront.setPower(leftFront);
            robot.leftBack.setPower(leftBack);
            robot.rightFront.setPower(rightFront);
            robot.rightBack.setPower(rightBack);}

        robot.leftFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

    }
}

