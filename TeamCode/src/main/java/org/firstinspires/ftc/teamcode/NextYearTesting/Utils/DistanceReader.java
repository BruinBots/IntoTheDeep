package org.firstinspires.ftc.teamcode.NextYearTesting.Utils;
import static org.firstinspires.ftc.teamcode.NextYearTesting.Utils.GlobalBot.bot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayDeque;

public class DistanceReader {
    public static ArrayDeque<Double> runningAverages = new ArrayDeque<>();

    public static void update() {
        double distance = bot.DistanceSensor.getDistance(DistanceUnit.INCH);
        int count = runningAverages.size();

        if (count == 5) {
            runningAverages.pollFirst();
        }
        runningAverages.addLast(distance);
    }

    public static double read() {
        double sum = 0;
        for (double d: runningAverages) {
            sum += d;
        }
        return sum / runningAverages.size();
    }
}
