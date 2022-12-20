package org.firstinspires.ftc.teamcode;

import static java.util.Collections.max;
import static java.util.Collections.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.HashMap;

@TeleOp(name = "DetectConeStack", group = "!!!!!!!!")
public class DetectConeStack extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SharedHardware.prepareHardware(hardwareMap);
        IOControl io = new IOControl(hardwareMap);
        DcMotor turret = SharedHardware.turret;
        Lift l = new Lift(hardwareMap);
        DistanceSensor distanceSensor = io.distSensorM;


        boolean initPosCompleted = false;
        boolean sweepCompleted = false;
        double min_dist = -1;

        int init_pos = -200; // "initialization" position (move to the right at first)
        int sweep_pos = 200; // final sweep position (sweep from right to left)
        int fudge = 10;

        ArrayList<Double> distances = new ArrayList<>();
        HashMap<Double, Double> encoderCounts = new HashMap<>();

        l.closeClaw();
        sleep(600);

        waitForStart();

        while (opModeIsActive()) {
            double turretPos = turret.getCurrentPosition();

            if (!initPosCompleted && !sweepCompleted) {
                turret.setTargetPosition(init_pos);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.5);

                if (turretPos < init_pos + fudge && turretPos > init_pos - fudge) {
                    initPosCompleted = true;
                } else {
                    // TODO
                }
            } else if (initPosCompleted && !sweepCompleted) {

                //sweep
                turret.setTargetPosition(sweep_pos);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.25);

                //record
                double distance = distanceSensor.getDistance(DistanceUnit.MM);
                distances.add(distance);
                encoderCounts.put(distance, turretPos);

                if (turretPos < sweep_pos + fudge && turretPos > sweep_pos - fudge) {
                    sweepCompleted = true;
                } else {
                    // TODO
                }
            } else {
                min_dist = min(distances);
                double target_turret_pos = encoderCounts.get(min_dist);

                turret.setTargetPosition((int) target_turret_pos);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.25);
            }

            /*
            if (initPosCompleted && sweepCompleted){
                min_dist = encoderCounts.get(min(distances));

                turret.setTargetPosition((int) min_dist);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.25);
            }
            else if (initPosCompleted) {
                int sweep_pos = 200;

                //sweep
                turret.setTargetPosition(sweep_pos);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(0.25);

                //record
                double distance = distanceSensor.getDistance(DistanceUnit.MM);
                distances.add(distance);
                encoderCounts.put(distance, turretPos);

                if (turretPos < sweep_pos + 10 && turretPos > sweep_pos - 10) {
                    sweepCompleted = true;
                } else {
                    //TODO
                }
            } else {
                //init pos
                int init_pos = -200;
                turret.setTargetPosition(init_pos);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(-0.5);

                if (turretPos < init_pos + 10 && turretPos > init_pos - 10) {
                    initPosCompleted = true;
                } else {
                    //TODO
                }
            }

            */

            telemetry.addData("min_dist", min_dist);
            telemetry.update();
        }
    }
}