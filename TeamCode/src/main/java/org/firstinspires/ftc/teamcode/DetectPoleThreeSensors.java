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
        INITIAL_SCAN,
        COMPENSATE_T,  // transition state
        COMPENSATE,
        START,  // transition state
        LOCKED
    }
    public ScanState scanState;
    public static final double COMPENSATE_SPEED = 0.2;
    public static final double SCAN_SPEED = 0.5;
    public static boolean initialScanDirection = true;  // INITIAL scan direction. true = right, false = left
    boolean currentScanDirection;

    double prevDistL = 0;
    double prevDistM = 0;
    double prevDistR = 0;

    // FL, FR, BL, BR
    public static final double[] STOP = { 0, 0, 0, 0 };
    public static final double[] TURN_LEFT = { -1, 1, -1, 1 };
    public static final double[] TURN_RIGHT = { 1, -1, 1, -1 };

    double[] motorPowers = STOP;

    public static final double DECREASE_LIMIT = -1000;  // mm

    public static double[] scaleVector(double[] vec, double sf) {
        double[] scaled = new double[vec.length];
        for (int i = 0; i < vec.length; i++) {
            scaled[i] = vec[i] * sf;
        }
        return scaled;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        IOControl ioControl = new IOControl(hardwareMap);  // connect to the hardware
        Telemetry mergedTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        scanState = ScanState.START;
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
                case START:
                    currentScanDirection = initialScanDirection;
                    motorPowers = scaleVector(currentScanDirection ? TURN_RIGHT : TURN_LEFT, SCAN_SPEED);
                    scanState = ScanState.INITIAL_SCAN;
                    // fall through
                case INITIAL_SCAN:
                    if (hitL || hitM || hitR) {
                        scanState = ScanState.COMPENSATE_T;
                        motorPowers = STOP;
                    }
                    break;
                case COMPENSATE_T:
                    currentScanDirection = !hitL;
                    scanState = ScanState.COMPENSATE;
                    motorPowers = scaleVector(currentScanDirection ? TURN_RIGHT : TURN_LEFT, COMPENSATE_SPEED);
                    // fall through
                case COMPENSATE:
                    if (hitM) {
                        scanState = ScanState.LOCKED;
                        motorPowers = STOP;
                    }
                    break;
                case LOCKED:
                    break;
            }

            // TODO set motor powers on actual motors

            prevDistL = currDistL;
            prevDistM = currDistM;
            prevDistR = currDistR;
        }
    }
}
