package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayDeque;

@Config
public class TeleDistanceDriver {
//    public SampleMecanumDriveCancelable drive;
    private Hardware bot;
    private Telemetry telemetry;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;

    public double target;
    public double tolerance;

//    public static double drivePower = 0.2;

    public static double farPower = 0.35;
    public static double midPower = 0.2;
    public static double nearPower = 0.15;

    public static double farThreshold = 12;
    public static double nearThreshold = 4;

    public double curPower = 0;

    public long lastTime;
    public static int timeout = 500;

    public void setTarget(double target, double tolerance) {
        this.target = target;
        this.tolerance = tolerance;
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

    public void loop() {
        lastTime = System.currentTimeMillis();

        updateRunningAverage();
        double curDist = getRunningAverage();

        // Drive the robot forward and backwards based on distance to the submersible

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

        curPower = -Math.copySign(power, error);

        bot.moveBotMecanum(-power, 0, 0, 1);
    }

    public boolean needsRunning() {
        if (target < 0.1 || tolerance < 0.01) { return false; }
        return (Math.abs(getRunningAverage() - target) > tolerance);
    }


    public TeleDistanceDriver(HardwareMap hardwareMap, Telemetry telemetry) {
        bot = new Hardware(hardwareMap);
        this.telemetry = telemetry;

        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();
    }
}
