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

    public static double drivePower = 0.2;

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
//        double sum = 0;
//        for (int i = 0; i < 5; i ++) {
//            sum += bot.DistanceSensor.getDistance(DistanceUnit.INCH);
//        }
//        double curDist = sum/5;
//
//        telemetry.addData("Current Distance", curDist);
//        dashTelemetry.addData("Current Distance", curDist);
//        telemetry.update();
//        dashTelemetry.update();
//
//        drive.update();
//        Pose2d startPose = drive.getPoseEstimate();
//        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                .forward(curDist - target)
//                .build();
//        drive.followTrajectorySequenceAsync(trajSeq);

        updateRunningAverage();
        double curDist = getRunningAverage();

        telemetry.addData("Current Distance", curDist);
        dashTelemetry.addData("Current Distance", curDist);

        // Drive the robot forward and backwards based on distance to the submersible

        double error = target - curDist;
        double power;

        if (error >= 0) {
            power = drivePower;
        }
        else {
            power = -drivePower;
        }

        bot.moveBotMecanum(-power, 0, 0, 1);

        updateRunningAverage();
    }

    public boolean needsRunning() {
        if (target < 0.1 || tolerance < 0.01) { return false; }
        return (Math.abs(getRunningAverage() - target) > tolerance);
    }

//    public boolean isBusy() { return drive.isBusy(); }

    public TeleDistanceDriver(HardwareMap hardwareMap, Telemetry telemetry) {
        bot = new Hardware(hardwareMap);
        this.telemetry = telemetry;

//        drive = new SampleMecanumDriveCancelable(hardwareMap);
//        drive.setPoseEstimate(new Pose2d(0, 0));
//        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();
    }
}
