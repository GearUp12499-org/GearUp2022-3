package org.firstinspires.ftc.teamcode;

import android.widget.EdgeEffect;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IOControl {
    public static final String DIST_SENSOR_L_HARDWARE_CONF_NAME = "distanceL";
    public static final String DIST_SENSOR_M_HARDWARE_CONF_NAME = "distance";
    public static final String DIST_SENSOR_R_HARDWARE_CONF_NAME = "distanceR";

    public DistanceSensor distSensorL;
    public DistanceSensor distSensorM;
    public DistanceSensor distSensorR;

    public IOControl(HardwareMap hardwareMap) {
        // FIXME attach distance sensors to robot
//        distSensorL = hardwareMap.get(DistanceSensor.class, DIST_SENSOR_L_HARDWARE_CONF_NAME);
        distSensorM = hardwareMap.get(DistanceSensor.class, DIST_SENSOR_M_HARDWARE_CONF_NAME);
//        distSensorR = hardwareMap.get(DistanceSensor.class, DIST_SENSOR_R_HARDWARE_CONF_NAME);
    }
}
