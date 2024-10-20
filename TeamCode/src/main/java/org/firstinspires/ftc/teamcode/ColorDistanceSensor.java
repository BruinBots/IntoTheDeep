package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorDistanceSensor {

    private final RevColorSensorV3 colorSensor;

    private double MAX_DISTANCE = 2.5;

    public NormalizedRGBA READING_COLORS;
    public double READING_DISTANCE;
    public float red;
    public float green;
    public float blue;

    boolean RED_VAL;
    boolean GREEN_VAL;
    boolean BLUE_VAL;

    public enum Colors {red, blue, yellow, none, nosample}

    public Colors color;

    public ColorDistanceSensor(RevColorSensorV3 sensor) {
        colorSensor = sensor;
    }

    public void getValues() {
        READING_COLORS = colorSensor.getNormalizedColors();
        READING_DISTANCE = colorSensor.getDistance(DistanceUnit.CM);
        red =  READING_COLORS.red;
        green = READING_COLORS.green;
        blue = READING_COLORS.blue;

        RED_VAL = red > 0.01;
        GREEN_VAL = green > 0.01;
        BLUE_VAL = blue > 0.01;
    }

    public void checkForColor() {
        if (READING_DISTANCE <= MAX_DISTANCE) {
            if ((RED_VAL && !GREEN_VAL && !BLUE_VAL) || (red > green && red > blue)) {
                color = Colors.red;
            } else if ((BLUE_VAL && !RED_VAL && !GREEN_VAL) || (blue > green && blue > red)) {
                color = Colors.blue;
            } else if ((RED_VAL && GREEN_VAL && !BLUE_VAL) || (red > blue && green > blue)) {
                color = Colors.yellow;
            } else {
                color = Colors.none;
            }
        } else {
            color = Colors.nosample;
        }
    }

    public void loop() {
        getValues();
        checkForColor();
    }

}