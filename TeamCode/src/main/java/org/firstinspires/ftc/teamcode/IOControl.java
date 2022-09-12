package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IOControl {
    public static final String DIST_SENSOR_HARDWARE_CONF_NAME = "...";
    public DistanceSensor distSensor;
    public IOControl(HardwareMap hardwareMap) {
        distSensor = hardwareMap.get(DistanceSensor.class, DIST_SENSOR_HARDWARE_CONF_NAME);
    }
}
