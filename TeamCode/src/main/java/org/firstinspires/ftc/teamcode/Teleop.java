package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SharedHardware.encoderLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRear;
import static org.firstinspires.ftc.teamcode.SharedHardware.encoderRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.frontRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.liftHorizontal;
import static org.firstinspires.ftc.teamcode.SharedHardware.prepareHardware;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearLeft;
import static org.firstinspires.ftc.teamcode.SharedHardware.rearRight;
import static org.firstinspires.ftc.teamcode.SharedHardware.runtime;
import static org.firstinspires.ftc.teamcode.SharedHardware.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleOp")
public class Teleop extends LinearOpMode {
    public Lift l;

    public final int TURRET_THRESHOLD = 800;
    public final int TURRET_DELTA = 1500; // STILL HAVE TO TEST
    public int turret_center;
    public boolean forward = true;
    public IOControl io;

    @Override
    public void runOpMode() throws InterruptedException {
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        io = new IOControl(hardwareMap);
        turret_center = turret.getCurrentPosition();

        waitForStart();
        while (opModeIsActive()) {
            drive();
            lift();
            turret();

            telemetry.addLine("Distance sensor:");
            telemetry.addData("Distance (mm)", io.distSensorM.getDistance(DistanceUnit.MM));
            telemetry.addLine("Odometry:");
            telemetry.addData("left", encoderLeft.getCurrentPosition());
            telemetry.addData("right", encoderRight.getCurrentPosition());
            telemetry.addData("f/b", encoderRear.getCurrentPosition());
            telemetry.addData("lift counts:", l.liftVertical1.getCurrentPosition());
            telemetry.addData("lift target:", l.targetVerticalCount);
            telemetry.update();
        }
    }

    //////////////////////////////////////////////////////////////////

    public void drive() {
        double speed = (0.25 + gamepad1.left_trigger * 0.75);
        double vX = 0; // forward/back
        double vY = 0; // left/right
        boolean useDPad = true;
        if (gamepad1.dpad_up) {
            vX += 1;
        } else if (gamepad1.dpad_down) {
            vX -= 1;
        } else if (gamepad1.dpad_left) {
            vY -= 1;
        } else if (gamepad1.dpad_right) {
            vY += 1;
        } else {
            useDPad = false;
        }
        if (useDPad) {
            double m1 = vX + vY;
            double m2 = vX - vY;
            double m3 = vX - vY;
            double m4 = vX + vY;
            driveMotor(speed * m1, speed * m2, speed * m3, speed * m4);
        } else {
            double left = -gamepad1.left_stick_y,
                    right = -gamepad1.right_stick_y;
            updateDirection();
            driveMotor(left, left, right, right);
        }
    }

    public void updateDirection() {
        if (gamepad1.right_trigger > 0.5) {
            if (gamepad1.x)
                forward = true;
            else if (gamepad1.y)
                forward = false;
        }
    }

    public void driveMotor(double lf, double lb, double rf, double rb) {
        if (!l.isExtended())
            lf = lb = rf = rb = 0;
        if (!forward) {
            lf = -lf;
            lb = -lb;
            rf = -rf;
            rb = -rb;
        }

        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        rearLeft.setPower(lb);
        rearRight.setPower(rb);
    }

    /////////////////////////////////////////////////////

    private boolean lastLeftBumper1 = false;
    private boolean last2DpadUp = false;
    private boolean last2DpadDown = false;
    private boolean last2Back = false;


    public void lift() {
        // TODO make dpad not go BRRRRRRRRRRRRRRRRRRRRRR
        /*
        if (RisingFallingEdges.isRisingEdge(gamepad2.back)) {
            l.liftVertical1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            l.liftVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            l.liftVertical1.setPower(Lift.POWER_DOWN / 2.0);
            l.liftVertical2.setPower(Lift.POWER_DOWN / 2.0); // slowly move down
        }
        if (RisingFallingEdges.isFallingEdge(gamepad2.back)) {
            l.liftVertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l.liftVertical1.setPower(Lift.POWER_UP);
            l.liftVertical1.setTargetPosition(0);
            l.liftVertical1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            l.liftVertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l.liftVertical2.setPower(0);
            l.liftVertical2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }*/

        if (gamepad2.back) {
            l.liftVertical1.setPower(Lift.POWER_DOWN * 0.5);
            l.liftVertical2.setPower(Lift.POWER_DOWN * 0.5);

            //return;  // Skip...
        }

        if (last2Back && !gamepad2.back) {
            l.liftVertical1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l.liftVertical2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            l.initLiftMotors(); // Reset encoders & stuff
        }
        last2Back = gamepad2.back;

        if (gamepad2.back) return;

        if (gamepad2.x){
                l.setVerticalTarget(2); //2
                l.update();
        }

        else if (gamepad2.b) {
            l.setHorizontalTargetManual(5);
            l.retract();
            l.update();
            int direction = sign(-turret.getCurrentPosition());
            turret.setPower(0.25 * direction);
            //l.setVerticalTarget(0);
            l.setVerticalTargetManual(175);
            runtime.reset();
            while (opModeIsActive() && !(Math.abs(turret.getCurrentPosition()) <= 20)) {
                if (Math.abs(turret.getCurrentPosition()) <= 10) {
                    turret.setPower(0);
                }
                if(runtime.seconds()>0.5)
                    l.update();
            }
            turret.setPower(0);

        } else if (gamepad2.a)
            l.setVerticalTarget(1); //2

            //l.setVerticalTarget(1);
        else if (gamepad2.y){
            //while(liftVertical1.getCurrentPosition()<4500){
                l.setVerticalTarget(3); //2
                l.update();
        }
        else if (gamepad2.dpad_up) {
            l.liftVertical1.setPower(Lift.POWER_UP * 0.8);
            l.liftVertical2.setPower(Lift.POWER_UP * 0.8);
//            l.setVerticalTargetManual(l.liftVertical1.getCurrentPosition());
//            l.inMotion = false;
        } else if (gamepad2.dpad_down) {
            l.liftVertical1.setPower(Lift.POWER_DOWN * 0.8);
            l.liftVertical2.setPower(Lift.POWER_DOWN * 0.8);
//            l.setVerticalTargetManual(l.liftVertical1.getCurrentPosition());
//            l.inMotion = false;
        }

        if ((last2DpadUp && !gamepad2.dpad_up) || (last2DpadDown && !gamepad2.dpad_down)) {
            l.setVerticalTargetManual(l.liftVertical1.getCurrentPosition());
        }

        last2DpadUp = gamepad2.dpad_up;
        last2DpadDown = gamepad2.dpad_down;
        if (gamepad2.right_bumper) {
            runtime.reset();
            while(liftHorizontal.getCurrentPosition()>20 && runtime.seconds()<2) {
                liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                liftHorizontal.setPower(-0.8);
            }
            liftHorizontal.setPower(0);
        } else if (gamepad2.left_bumper) {
            liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftHorizontal.setPower(0.8);
            l.update();
        } else if (gamepad2.right_trigger > 0.2) {
            liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftHorizontal.setPower(-0.2);
            l.update();
        } else if (gamepad2.left_trigger > 0.2 ) {
            liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftHorizontal.setPower(0.2);
            l.update();
        }else{
            liftHorizontal.setPower(0);
        }


        // CLAW
        if (gamepad1.left_bumper && !lastLeftBumper1) {
            l.closeClaw();
            sleep(600);
            l.moveVertical(300);
        } else if (gamepad1.right_bumper) l.openClaw();


        lastLeftBumper1 = gamepad1.left_bumper;
        l.update();

        telemetry.addData("horizontal slider", l.liftHorizontal.getCurrentPosition());
        telemetry.addData("horizontal slider to", l.liftHorizontal.getTargetPosition());
        telemetry.addData("sensor read", io.distSensorM.getDistance(DistanceUnit.MM));
    }

    private int sign(int i) {
        return Integer.compare(i, 0);
    }

    ////////////////////////////////////////////////////////////////////

    public void turret() throws InterruptedException {
        int b = 0;
        if (l.liftVertical1.getCurrentPosition() < 100)//TURRET_THRESHOLD)
            return;

        double speed = gamepad2.left_stick_x * 0.5; //Math.pow(gamepad2.left_stick_x, 2);
        int now = turret.getCurrentPosition() - turret_center;
        //if ((speed < 0 && now > -TURRET_DELTA) || (speed > 0 && now < TURRET_DELTA))
        turret.setPower(speed);

        if (gamepad1.b) {
            l.setVerticalTargetManual(Math.max(l.liftVertical1.getCurrentPosition(), Lift.inEnc(14)));
            l.waitLift(this);
            runtime.reset();
            while (io.distSensorM.getDistance(DistanceUnit.MM) > 300 && Math.abs(turret.getCurrentPosition()) < 1200 && runtime.seconds()<2) {
                turret.setPower(0.35);
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            turret.setPower(0);
        } else if (gamepad1.x) {
            l.setVerticalTargetManual(Math.max(l.liftVertical1.getCurrentPosition(), Lift.inEnc(14)));
            l.waitLift(this);
            runtime.reset();
            while (io.distSensorM.getDistance(DistanceUnit.MM) > 300 && Math.abs(turret.getCurrentPosition()) < 1200 && runtime.seconds()<2) {
                turret.setPower(-0.35);
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
            }
            turret.setPower(0);
        }
        //else
        //turret.setPower(0);
        // if(turret.getCurrentPosition() == 0)
        //   turret.setPower(0);
        /*if(gamepad2.b){
            //runtime.reset();
            b =1;
            if(turret.getCurrentPosition() > 0){
                turret.setTargetPosition(turret_center);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.5);
            }
            else if(turret.getCurrentPosition() < 0){
                turret.setTargetPosition(0);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.5);
            }
        }*/
        telemetry.addData("turret center:", turret_center);
        telemetry.addData("turret position:", now);
        telemetry.addData("turret speed:", speed);

    }
}
