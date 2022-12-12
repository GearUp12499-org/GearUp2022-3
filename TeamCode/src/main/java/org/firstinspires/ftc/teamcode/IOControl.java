package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IOControl {
    public static final String DIST_SENSOR_M_HARDWARE_CONF_NAME = "distance";

    public DistanceSensor distSensorM;

    public IOControl(HardwareMap hardwareMap) {
        distSensorM = hardwareMap.get(DistanceSensor.class, DIST_SENSOR_M_HARDWARE_CONF_NAME);
    }
}
