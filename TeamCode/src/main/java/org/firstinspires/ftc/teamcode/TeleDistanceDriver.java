package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TeleDistanceDriver {
    public SampleMecanumDriveCancelable drive;
    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;

    public double target;
    public double tolerance;

    public void setTarget(double target, double tolerance) {
        this.target = target;
        this.tolerance = tolerance;
    }

    public void loop() {
        double sum = 0;
        for (int i = 0; i < 5; i ++) {
            sum += bot.DistanceSensor.getDistance(DistanceUnit.INCH);
        }
        double curDist = sum/5;

        telemetry.addData("Current Distance", curDist);
        dashTelemetry.addData("Current Distance", curDist);
        telemetry.update();
        dashTelemetry.update();

        drive.update();
        Pose2d startPose = drive.getPoseEstimate();
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(curDist - target)
                .build();
        drive.followTrajectorySequenceAsync(trajSeq);
    }

    public boolean needsRunning() {
        double sum = 0;
        for (int i = 0; i < 5; i ++) {
            sum += bot.DistanceSensor.getDistance(DistanceUnit.INCH);
        }
        double curDist = sum/5;
        return (Math.abs(curDist - target) > tolerance);
    }

    public boolean isBusy() { return drive.isBusy(); }

    public TeleDistanceDriver(HardwareMap hardwareMap) {
        bot = new Hardware(hardwareMap);

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0));

        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();
    }
}
