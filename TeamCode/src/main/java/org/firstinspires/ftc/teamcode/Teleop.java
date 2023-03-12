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
import org.firstinspires.ftc.teamcode.lib.LinearCleanupOpMode;
import org.firstinspires.ftc.teamcode.snap.MatchTimer;
import org.firstinspires.ftc.teamcode.snap.SnapRunner;

@TeleOp(name = "TeleOp")
public class Teleop extends LinearCleanupOpMode {
    public Lift l;

    public final int TURRET_THRESHOLD = 800;
    public final int TURRET_DELTA = 1500; // STILL HAVE TO TEST
    public int turret_center;
    public boolean forward = true;
    public IOControl io;

    @Override
    public void cleanup() {
        if (frontLeft != null) {
            frontLeft.setPower(0);
        }
        if (frontRight != null) {
            frontRight.setPower(0);
        }
        if (rearLeft != null) {
            rearLeft.setPower(0);
        }
        if (rearRight != null) {
            rearRight.setPower(0);
        }
        if (l != null) {
            l.liftVertical1.setPower(0);
            l.liftVertical2.setPower(0);
            l.liftHorizontal.setPower(0);
        }
        if (turret != null) {
            turret.setPower(0);
        }
    }

    @Override
    public void main() throws InterruptedException {
        prepareHardware(hardwareMap);
        l = new Lift(hardwareMap);
        io = new IOControl(hardwareMap);
        turret_center = turret.getCurrentPosition();
        SnapRunner snapRunner = new SnapRunner();
        snapRunner.addSnap(new MatchTimer(telemetry));

        waitForStart();
        snapRunner.init();
        while (opModeIsActive()) {
            drive();
            lift();
            turret();
            autoScore();
            //l.update2();
            l.liftVertical1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            l.liftVertical2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            telemetry.addLine("Distance sensor:");
            telemetry.addData("Distance (mm)", io.distSensorM.getDistance(DistanceUnit.MM));
            telemetry.addLine("Odometry:");
            telemetry.addData("left", encoderLeft.getCurrentPosition());
            telemetry.addData("right", encoderRight.getCurrentPosition());
            telemetry.addData("f/b", encoderRear.getCurrentPosition());
            telemetry.addData("lift counts:", l.liftVertical1.getCurrentPosition());
            telemetry.addData("lift target:", l.targetVerticalCount);
            telemetry.addData("turret pos:", turret.getCurrentPosition());
            snapRunner.loop();
            telemetry.update();
        }
        snapRunner.finish();
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
        int hTargPos = 0;
        if(gamepad1.start){
            hTargPos = liftHorizontal.getCurrentPosition();
        }
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
        //int count = (int)(l.liftVertical1.getCurrentPosition()*537.7/384.5);

        if (gamepad2.x){
            //l.setVerticalTarget(2); //2
            l.setVerticalTargetManual(2350); //2

            //l.update();
        }

        else if (gamepad2.b) {
            l.setHorizontalTargetManual(5);
            l.retract();
            l.update();
            int direction = sign(-turret.getCurrentPosition());
            turret.setPower(0.5 * direction);
            //l.setVerticalTarget(0);
            l.setVerticalTargetManual(175);
            runtime.reset();
            while (opModeIsActive() && !(Math.abs(turret.getCurrentPosition()) <= 20)) {
                if (Math.abs(turret.getCurrentPosition()) <= 100) {
                    turret.setPower(0.12*direction);
                }
                if (Math.abs(turret.getCurrentPosition()) <= 10) {
                    turret.setPower(0);
                }
                if(runtime.seconds()>0.5)
                    l.update();
                rearLeft.setPower(0);
                rearRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            turret.setPower(0);

        } else if (gamepad2.a){
            l.setVerticalTargetManual(1150); //2

        }
            //l.setVerticalTarget(1); //2

            //l.setVerticalTarget(1);
        else if (gamepad2.y){
            //while(liftVertical1.getCurrentPosition()<4500){
                l.setVerticalTargetManual(3315); //3475
                l.update();
        }

        else if (gamepad2.dpad_up) {
            l.comp = false;
          //  l.setVerticalTargetManual(count+2000);
            l.liftVertical1.setPower(Lift.POWER_UP * 0.8);
            l.liftVertical2.setPower(Lift.POWER_UP * 0.8);
            l.setVerticalTargetManual(l.liftVertical1.getCurrentPosition());
            ;

        } else if (gamepad2.dpad_down && l.liftVertical1.getCurrentPosition()>450) {
            l.comp = false;
            l.liftVertical1.setPower(Lift.POWER_DOWN * 0.2);
            l.liftVertical2.setPower(Lift.POWER_DOWN * 0.2);
            l.setVerticalTargetManual(l.liftVertical1.getCurrentPosition());


        }
       // l.comp = true;
       if (!gamepad2.dpad_down && !gamepad2.dpad_up) {
            l.comp = true;
            l.update();
        }

        if ((last2DpadUp && !gamepad2.dpad_up) || (last2DpadDown && !gamepad2.dpad_down)) {
            l.setVerticalTargetManual(l.liftVertical1.getCurrentPosition());
        }

        last2DpadUp = gamepad2.dpad_up;
        last2DpadDown = gamepad2.dpad_down;
        if (gamepad2.right_bumper) {
            runtime.reset();
            while(liftHorizontal.getCurrentPosition()>10 && runtime.seconds()<2) {
                if(liftHorizontal.getCurrentPosition()<80){
                    liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftHorizontal.setPower(-0.2);
                }
                else{
                    liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftHorizontal.setPower(-0.8);
                }
                rearLeft.setPower(0);
                rearRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            liftHorizontal.setPower(0);
        } else if (gamepad2.left_bumper) {
            while(liftHorizontal.getCurrentPosition()<hTargPos) {
                liftHorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftHorizontal.setTargetPosition(hTargPos);
                liftHorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftHorizontal.setPower(0.11);
                rearLeft.setPower(0);
                rearRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
           /* double x = liftHorizontal.getCurrentPosition();
            double vLiftEti = 759/5.25;//encoder count to inch
            int targ = 0;
            liftHorizontal.setPower(0.2);
            if(l.liftVertical1.getCurrentPosition()<500){
                double h = (4.93 - 3.4 - 0.000658*(x)-0.00000324*(x)*(x));
                targ = (int)((1.9-h)*vLiftEti);
                l.setVerticalTargetManual(targ);

            }
            liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftHorizontal.setPower(0.8);
            l.update();*/
        } else if (gamepad2.right_trigger > 0.2) {
            liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftHorizontal.setPower(-0.4);
            l.update();
        } else if (gamepad2.left_trigger > 0.2) {
            double x = liftHorizontal.getCurrentPosition();
            double vLiftEti = 759/5.25;//encoder count to inch
            int targ = 0;
            liftHorizontal.setPower(0.18); //0.2
            if(l.liftVertical1.getCurrentPosition()<500){
                // sag correction
                double h = (4.93 - 3.4 - 0.000658*(x)-0.00000324*(x)*(x));
                targ = (int)((2.4-h)*vLiftEti); //2.9
                l.setVerticalTargetManual(targ);

            }
            liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if(gamepad2.left_trigger<0.99)
                if(l.liftVertical1.getCurrentPosition()>1000){
                    liftHorizontal.setPower(0.20);
                }
                else
                    liftHorizontal.setPower(0.20);//35
            else if(gamepad2.left_trigger>0.99)
                liftHorizontal.setPower(0.6);
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
    public void autoScore(){ //automating scoring
        /*
        basically the way this is going to work is the following
        1. drivers score one manually
            a. when ready to grab cup from stack, driver 1 hits gamepad1.start
            b. when right above the pole ready to deliver, driver 1 hits gamepad1.back
        2. driver 1 can hit y, and it will reset the robots position that is ready to pick up cup
        3. driver 1 will manually close claw
        4. driver 1 will then hit a, which will go from grabbing cone position, to right above pole position
        This cycle repeats on an on, this will probably be revised/optimized as we practice it
        --JJ
         */
        /*
        int turrStackPos =0;
        int hLiftStackPos= 0;
        int turrPolePos = 0;
        int hLiftPolePos = 0;
        int a = 0;

        //setting encoder count values
        if(gamepad1.start){
            turrStackPos = turret.getCurrentPosition();
            hLiftStackPos = liftHorizontal.getCurrentPosition();
        }
        else if(gamepad1.back){
            turrPolePos = turret.getCurrentPosition();
            hLiftPolePos = liftHorizontal.getCurrentPosition();
            if(turrPolePos>0) //this means that you are dropping off to a pole to the left of your robot, if turret side is considered front
                a = 1;
            else if(turrPolePos<0)
                a = -1; // this means that you are dropping off to a pole to the right of your robot
        }

        //getting to cone stack
        if(gamepad1.y){
            double vLiftEti = 759/5.25;//encoder count to inch
            if(l.liftVertical1.getCurrentPosition()>2000){
                if(a == 1) {
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setTargetPosition(turrStackPos); //750
                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turret.setPower(-0.8); //0.3
                    sleep(500);

                    //lowers vertical lift to cone stack and extends out horizontal lift to stack
                    l.setVerticalTargetManual(100);
                    l.setHorizontalTargetManual(825);
                    while (l.liftVertical1.getCurrentPosition() > (100)) {
                        l.liftVertical1.setPower(-0.6);
                        l.liftVertical2.setPower(-0.6);
                    }
                }
                else if(a == -1){
                    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    turret.setTargetPosition(turrStackPos); //750
                    turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    turret.setPower(0.8); //0.3
                    sleep(500);

                    //lowers vertical lift to cone stack and extends out horizontal lift to stack
                    l.setVerticalTargetManual(100);
                    l.setHorizontalTargetManual(825);
                    while (l.liftVertical1.getCurrentPosition() > (100)) {
                        l.liftVertical1.setPower(-0.6);
                        l.liftVertical2.setPower(-0.6);
                    }
                    double h = (4.93 - 3.4 - 0.000658*(hLiftStackPos)-0.00000324*(hLiftStackPos)*(hLiftStackPos));
                    int targ = (int)((1.9-h)*vLiftEti);
                    l.setVerticalTargetManual(targ);
                    while(liftHorizontal.getCurrentPosition()<hLiftStackPos){
                        liftHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        liftHorizontal.setPower(0.2);
                        l.update();
                    }
                }
            }
        }
*/

    }

    public void turretAuto(){

    }

    //------------------------------------------------------
    public void turret() throws InterruptedException {
        int b = 0;
        if (l.liftVertical1.getCurrentPosition() < 100)//TURRET_THRESHOLD)
            return;

        double speed = gamepad2.left_stick_x * 0.5; //Math.pow(gamepad2.left_stick_x, 2);
        int now = turret.getCurrentPosition() - turret_center;
        //if ((speed < 0 && now > -TURRET_DELTA) || (speed > 0 && now < TURRET_DELTA))
        turret.setPower(speed);

        if (gamepad1.b) {
            l.setVerticalTargetManual(Math.max(l.liftVertical1.getCurrentPosition(), Lift.inEnc(10))); //14
            runtime.reset();
            //200
            while (io.distSensorM.getDistance(DistanceUnit.MM) > 260 && Math.abs(turret.getCurrentPosition()) < 1200 && runtime.seconds()<2) {
                turret.setPower(0.30); //0.35
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
                l.update();
                rearLeft.setPower(0);
                rearRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
            }
            turret.setPower(0);
        } else if (gamepad1.x) {
            l.setVerticalTargetManual(Math.max(l.liftVertical1.getCurrentPosition(), Lift.inEnc(10)));
            runtime.reset();
            while (io.distSensorM.getDistance(DistanceUnit.MM) > 260 && Math.abs(turret.getCurrentPosition()) < 1200 && runtime.seconds()<2) {
                turret.setPower(-0.30); //0.35
                telemetry.addData("distance:", io.distSensorM.getDistance(DistanceUnit.CM));
                telemetry.update();
                l.update();
                rearLeft.setPower(0);
                rearRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
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
