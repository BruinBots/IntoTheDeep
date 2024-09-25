package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hardware {

    RevColorSensorV3 colorSensor;
    ColorDistanceSensor colorDistanceSensor;
    public Hardware(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color_sensor");
        colorDistanceSensor = new ColorDistanceSensor(colorSensor);
    }

}
