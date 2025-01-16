package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.farPower;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.farThreshold;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.midPower;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.nearPower;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.nearThreshold;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.TeleDistanceDriver;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;

import java.util.ArrayDeque;

public class AutoDistance {
    public Hardware bot;
    public LinearOpMode mode;
    public SampleMecanumDriveCancelable drive;

    public Telemetry telemetry;
    public Telemetry dashTelemetry;

    public AutoDistance(Hardware bot, LinearOpMode mode, SampleMecanumDriveCancelable drive, Telemetry telemetry, Telemetry dashTelemetry) {
        this.bot = bot;
        this.mode = mode;
        this.drive = drive;

        this.telemetry = telemetry;
        this.dashTelemetry = dashTelemetry;
    }

    public Pose2d drive2distance(double target, double tolerance) {
        // Scaling factor for distance to submersible into drive commands
        updateRunningAverage();
        double curDist = getRunningAverage();

        if (target == 0 || tolerance == 0) {
            return null;
        }

        while (Math.abs(curDist - target) > tolerance) {
            if (mode.isStopRequested()) {
                return null;
            }

            updateRunningAverage();
            curDist = getRunningAverage();

            while (Double.isNaN(curDist)) {
                updateRunningAverage();
                curDist = getRunningAverage();
            }

            telemetry.addData("Current Distance", curDist);
            dashTelemetry.addData("Current Distance", curDist);
            telemetry.update();
            dashTelemetry.update();

            double error = target - curDist;
            double absError = Math.abs(error);
            double power;

            if (absError > farThreshold) {
                power = farPower;
            } else if (absError > nearThreshold) {
                power = midPower;
            } else {
                power = nearPower;
            }

            bot.moveBotMecanum(-Math.copySign(power, error), 0, 0, 1);
        }
        bot.moveBotMecanum(0, 0, 0, 0);
        return drive.getPoseEstimate();
    }

    public ArrayDeque<Double> runningAverages = new ArrayDeque<>();

    public void updateRunningAverage() {
        double distance = bot.DistanceSensor.getDistance(DistanceUnit.INCH);
        int count = runningAverages.size();

        if (count == 5) {
            runningAverages.pollFirst();
        }
        runningAverages.addLast(distance);
    }

    public double getRunningAverage() {
        double sum = 0;
        for (double d: runningAverages) {
            sum += d;
        }
        return sum / runningAverages.size();
    }
}
