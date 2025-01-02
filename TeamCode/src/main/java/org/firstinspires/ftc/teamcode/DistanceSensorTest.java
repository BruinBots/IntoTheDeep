package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class DistanceSensorTest {
    private final ModernRoboticsI2cRangeSensor distanceSensor;
    public double READING_DISTANCE;

    public DistanceSensorTest(ModernRoboticsI2cRangeSensor sensor) {
        distanceSensor = sensor;
    }

    public void getValues() {
        READING_DISTANCE = distanceSensor.getDistance(DistanceUnit.CM);
    }

    public void loop() {
        getValues();
    }
}