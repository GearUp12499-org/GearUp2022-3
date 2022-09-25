package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



/*

# wlog, turret is turning to the right

while no sensor senses anything
    keep moving turret

stop turret

fix_direction = right
if left sensor sees
    fix_direction = left

set motor speed really slow
while middle sensor is not the closest??? / only one seeing pole
    move turret toward fix_direction

*/


@TeleOp(name = "Detect Pole Three Sensors", group = "!!!!!!!!")
public class DetectPoleThreeSensors extends LinearOpMode {

    final boolean rotationDirectionIsRight = false;

    double prevDistL = 0;
    double prevDistM = 0;
    double prevDistR = 0;

    public static final double LIMIT = 1000;  // mm

    boolean storeHitL = false;
    boolean storeHitM = false;
    boolean storeHitR = false;

    int fixDirection = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        IOControl ioControl = new IOControl(hardwareMap);  // connect to the hardware
        Telemetry mergedTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while (opModeIsActive()) {

            // CHECK DISTANCE SENSORS
            double currDistL = ioControl.distSensorL.getDistance(DistanceUnit.MM);
            double currDistM = ioControl.distSensorM.getDistance(DistanceUnit.MM);
            double currDistR = ioControl.distSensorR.getDistance(DistanceUnit.MM);

            double deltaL = currDistL - prevDistL;
            double deltaM = currDistM - prevDistM;
            double deltaR = currDistR - prevDistR;

            if (Math.abs(deltaL) >= LIMIT && deltaL < 0) storeHitL = true;
            if (Math.abs(deltaM) >= LIMIT && deltaM < 0) storeHitM = true;
            if (Math.abs(deltaR) >= LIMIT && deltaR < 0) storeHitR = true;

            if (storeHitL || storeHitM || storeHitR) {
                // stop turret
                if(rotationDirectionIsRight){
                    fixDirection = -1;
                    if(storeHitL) fixDirection = 1;
                } else{
                    fixDirection = 1;
                    if(storeHitR) fixDirection = -1;
                }

                // change turret direction toward fixDirection
            }

            if(fixDirection != 0){
                // set speed really slow
                if(storeHitM){
                    // terminate
                }
            }

            prevDistL = currDistL;
            prevDistM = currDistM;
            prevDistR = currDistR;

        }
    }
}
