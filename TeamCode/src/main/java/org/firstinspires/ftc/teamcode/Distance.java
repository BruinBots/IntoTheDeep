package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance {
    private final ModernRoboticsI2cRangeSensor distanceSensor;

    public Distance(ModernRoboticsI2cRangeSensor sensor) {
        distanceSensor = sensor;
    }

    public double getValue() {
        return(distanceSensor.getDistance(DistanceUnit.INCH));
    }
}