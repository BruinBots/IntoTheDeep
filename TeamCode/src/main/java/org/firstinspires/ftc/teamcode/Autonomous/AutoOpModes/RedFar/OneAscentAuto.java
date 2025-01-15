package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedFar;

import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.farPower;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.farThreshold;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.midPower;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.nearPower;
import static org.firstinspires.ftc.teamcode.TeleDistanceDriver.nearThreshold;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.AprilReader;
import org.firstinspires.ftc.teamcode.ChamberPlacer;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.TeleDistanceDriver;
import org.firstinspires.ftc.teamcode.Viper;
import org.firstinspires.ftc.teamcode.WallPicker;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayDeque;

@Config
@Autonomous(name = "OneAscentAuto", preselectTeleOp = "Main Teleop")
public class OneAscentAuto extends LinearOpMode {

    public static int START_X = -36;
    public static int START_Y = -60;
    public static int START_ANGLE = 90;
    public static int MID_ANGLE = START_ANGLE;
    public static int INVERTED_START_ANGLE = 270;
    public static int ASCENT_ANGLE = -90;

    public Pose2d startPose;
    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDriveCancelable drive;

    public static int AX = -48;
    public static int AY = -10;

    public static int BX = -30;
    public static int BY = -6;

    public void tele(String caption, String message) {
        telemetry.addData(caption, message);
        dashTelemetry.addData(caption, message);
        telemetry.update();
        dashTelemetry.update();
    }

    public void status(String message) {
        tele("Status", message);
    }

    public void doPos() {
        tele("X", "" + drive.getPoseEstimate().getX());
        tele("Y", "" + drive.getPoseEstimate().getY());
        tele("Heading", "" + drive.getPoseEstimate().getHeading());
    }

    public void drive2distance(double target, double tolerance) {
        // Scaling factor for distance to submersible into drive commands
        updateRunningAverage();
        double curDist = getRunningAverage();

        while (Math.abs(curDist - target) > tolerance) {
            if (isStopRequested()) {
                return;
            }

            updateRunningAverage();
            curDist = getRunningAverage();

            telemetry.addData("Current Distance", curDist);
            dashTelemetry.addData("Current Distance", curDist);
            telemetry.update();
            dashTelemetry.update();

            // Drive the robot forward and backwards based on distance to the submersible

//            double error = target - curDist;
//            double power;
//
//            if (error >= 0) {
//                power = TeleDistanceDriver.drivePower;
//            }
//            else {
//                power = -TeleDistanceDriver.drivePower;
//            }
//
//            bot.moveBotMecanum(-power, 0, 0, 1);
//            startPose = drive.getPoseEstimate();
//
//            updateRunningAverage();

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
            startPose = drive.getPoseEstimate();
        }
        bot.moveBotMecanum(0, 0, 0, 0);
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

    @Override
    public void runOpMode() {
        bot = new Hardware(hardwareMap);
        bot.init(false);

        // Dashboard telemetry
        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();

        // RR
        startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANGLE));
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(startPose);

        waitForStart();

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(AX, AY))
                .addDisplacementMarker(20, () -> {
                    bot.viper.move(ChamberPlacer.downViper, Viper.Sides.LEFT);
                })
                .turn(Math.toRadians(ASCENT_ANGLE))
                .lineToConstantHeading(new Vector2d(BX, BY))
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            drive.update();
            if (isStopRequested()) {
                return;
            }
        }

        drive2distance(1, 0.5);
    }
}
