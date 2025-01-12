package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedFar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.AprilReader;
import org.firstinspires.ftc.teamcode.ChamberPlacer;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Viper;
import org.firstinspires.ftc.teamcode.WallPicker;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayDeque;

@Config
@Autonomous(name = "Deprecated:  FasterAuto", preselectTeleOp = "Main Teleop")
public class FasterAuto extends LinearOpMode {

    public static boolean blue = false;
    public static int START_X = 10;
    public static int START_ANGLE = blue ? 270 : 90;
    public static int MID_ANGLE = START_ANGLE;
    public static int INVERTED_START_ANGLE = blue ? 90 : 270;
    public static int SUBMERSIBLE_X = 0;
    public static int SUBMERSIBLE_X2 = 4;
    public static int APRIL_TIME = 1000;
    public static boolean doApril = false;
    private static int bluef = blue ? 1 : -1;
    public static int START_Y = 60 * bluef;
    public static int SUBMERSIBLE_Y = 30 * bluef;
    public static int OBSERVATION_X = -45 * bluef;
    public static int OBSERVATION_Y = 54 * bluef;
    public static int MID_X = -24 * bluef;
    public static int MID_Y = 48 * bluef;
    public Pose2d startPose;
    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDrive drive;
    private AprilReader reader;

    public static int SAMPLE0X = bluef*-36;
    public static int SAMPLE0Y = bluef*12;
    public static int SAMPLEY = bluef*6;
    public static int SAMPLE1X = bluef*-46;
    public static int SAMPLE2X = bluef*-54;
    public static int SAMPLE3X = bluef*-62;
    public static int SAMPLE_PUSH = 48;
    public static int SAMPLE_PULL = 36;

    public static double XOFFSET = 7;
    public static double YOFFSET = 3.5;

    public void drive2distance(double target, double tolerance) {
        double curDist = getRunningAverage();
        while (Math.abs(curDist - target) > tolerance) {
            curDist = getRunningAverage();
            telemetry.addData("Current Distance", curDist);
            dashTelemetry.addData("Current Distance", curDist);
            telemetry.update();
            dashTelemetry.update();
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(curDist - target)
                    .build();
            drive.followTrajectorySequenceAsync(trajSeq);
            startPose = trajSeq.end();
            while (drive.isBusy()) {
                if (isStopRequested()) {
                    return;
                }
                updateRunningAverage();
            }
        }
    }

    public void runApril() {
        if (doApril) {
            double[] april;
            long startTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - startTime < APRIL_TIME) && !isStopRequested()) {
                bot.frames.loop();
                april = reader.read();
                if (april.length == 3) {
                    double x = april[0];
                    double y = april[1];
                    double yaw = april[2];
                    double heading = yaw + Math.atan(YOFFSET/XOFFSET);
                    startPose = new Pose2d(x, y, heading);
                    drive.setPoseEstimate(startPose);
                    status("Pose Updated with april tags");
                }
            }
        }
    }

    public void aprilLoop() {
        double[] april = reader.read();
        if (april.length == 3) {
            double x = april[0];
            double y = april[1];
            double yaw = april[2];
            double heading = yaw + Math.atan(YOFFSET/XOFFSET);
            startPose = new Pose2d(x, y, heading);
            drive.setPoseEstimate(startPose);
            status("Pose Updated with april tags");
        }
    }

    public ArrayDeque<Double> runningAverages = new ArrayDeque<>();

    public void updateRunningAverage() {
        double distance = bot.DistanceSensor.getDistance(DistanceUnit.INCH);
        int count = runningAverages.size();

        if (count == 10) {
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

    @Override
    public void runOpMode() {
        bot = new Hardware(hardwareMap);
        bot.init(false);

        // Dashboard telemetry
        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();

        // RR
        startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANGLE));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        // April Tags
        reader = new AprilReader(hardwareMap);

        // Wait for start
        while (!isStarted()) {
            if (isStopRequested()) {
                return;
            }
        }

        // lift before start
        bot.frames.lift();
        bot.frames.loop();
        status("Lifting arm...");
//        while (bot.frames.isBusy() && !isStopRequested()) {
//            bot.frames.loop();
//        }

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(6)
                .splineTo(new Vector2d(SUBMERSIBLE_X, SUBMERSIBLE_Y), Math.toRadians(START_ANGLE))
                .addDisplacementMarker(2, () -> {
                    // Start lifting viper
                    bot.viper.move(3050, Viper.Sides.LEFT);
                })
                .build();
        status("Going to submersible...");
        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            if (isStopRequested()) {
                return;
            }
            updateRunningAverage();
        }

        bot.frames.topPole();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }

        status("Aligning with distance sensor...");
        drive2distance(ChamberPlacer.chamberPlacerDistance, ChamberPlacer.chamberPlacerTolerance);

        status("Placing specimen on high chamber...");
        bot.frames.topSpecimen();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .addDisplacementMarker(2, () -> {
                    bot.viper.move(180, Viper.Sides.LEFT);
                })
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(OBSERVATION_X, OBSERVATION_Y), Math.toRadians(INVERTED_START_ANGLE))
                .forward(4)
                .build();
        status("Moving to observation zone...");
        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            if (isStopRequested()) {
                return;
            }
            updateRunningAverage();
        }

        status("Aligning with distance sensor...");
        drive2distance(WallPicker.wallPickerDistance, WallPicker.wallPickerTolerance);

        status("Picking up specimen...");
        bot.basket.setClosed();
        bot.frames.afterWall();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .turn(Math.toRadians(-90))
                .splineTo(new Vector2d(SUBMERSIBLE_X2, SUBMERSIBLE_Y), Math.toRadians(START_ANGLE))
                .addDisplacementMarker(20, () -> {
                    bot.viper.move(ChamberPlacer.startViper, Viper.Sides.LEFT);
                })
                .build();
        status("Going to submersible...");
        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        bot.frames.topPole();
        while (drive.isBusy()) {
            bot.frames.loop();
            updateRunningAverage();
            if (isStopRequested()) {
                return;
            }
        }

        status("Aligning with distance sensor...");
        drive2distance(ChamberPlacer.chamberPlacerDistance, ChamberPlacer.chamberPlacerTolerance);

        status("Placing specimen on high chamber...");
        bot.frames.topSpecimen();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .turn(Math.toRadians(-90))
                .forward(6)
                .splineTo(new Vector2d(SAMPLE0X, SAMPLE0Y), Math.toRadians(START_ANGLE))
                .splineTo(new Vector2d(SAMPLE1X, SAMPLEY), Math.toRadians(START_ANGLE))
                .back(SAMPLE_PUSH)
                .forward(SAMPLE_PULL)
                .splineTo(new Vector2d(SAMPLE2X, SAMPLEY), Math.toRadians(START_ANGLE))
                .back(SAMPLE_PUSH)
                .forward(SAMPLE_PULL)
                .splineTo(new Vector2d(SAMPLE3X, SAMPLEY), Math.toRadians(START_ANGLE))
                .back(SAMPLE_PUSH)
                .build();
        drive.followTrajectorySequence(trajSeq);
        startPose = trajSeq.end();

        status("Lowering arm...");
        bot.frames.zeroArm();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }

        status("Lowering viper...");
        bot.frames.zeroBasket();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }
    }
}
