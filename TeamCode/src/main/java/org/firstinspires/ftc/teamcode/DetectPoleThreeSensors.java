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

    public enum ScanState {
        INITIAL,
        COMPENSATE_T,  // transition state
        COMPENSATE,
        LOCKED
    }
    public ScanState scanState;
    public static final double COMPENSATE_SPEED = 0.1;
    public static boolean initialScanDirection = true;  // INITIAL scan direction. true = right, false = left
    boolean currentScanDirection;

    double prevDistL = 0;
    double prevDistM = 0;
    double prevDistR = 0;

    // FL, FR, BL, BR
    double[] motorPowers = {0, 0, 0, 0};
    public static final double[] toTurnLeft = { /* TODO input motor powers, 1/0 only */ };
    public static final double[] toTurnRight = { /* TODO input motor powers, 1/0 only */ };

    public static final double DECREASE_LIMIT = -1000;  // mm

    @Override
    public void runOpMode() throws InterruptedException {
        currentScanDirection = initialScanDirection;
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

            boolean hitL = deltaL <= DECREASE_LIMIT;
            boolean hitM = deltaM <= DECREASE_LIMIT;
            boolean hitR = deltaR <= DECREASE_LIMIT;

            switch (scanState) {
                case INITIAL:
                    if (hitL || hitM || hitR) {
                        scanState = ScanState.COMPENSATE_T;
                        // TODO stop motors
                    }
                case COMPENSATE_T:
                    currentScanDirection = !hitL;
                    scanState = ScanState.COMPENSATE;
                    // fall through
                case COMPENSATE:
                    if (hitM) {
                        scanState = ScanState.LOCKED;
                        // TODO stop motors
                    }
                    break;
                case LOCKED:
                    break;
            }
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
