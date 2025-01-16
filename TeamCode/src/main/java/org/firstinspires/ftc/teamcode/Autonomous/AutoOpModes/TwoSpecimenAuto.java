package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes;

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
@Autonomous(name = "B-Comp: TwoSpecimenAuto", preselectTeleOp = "Main Teleop")
public class TwoSpecimenAuto extends LinearOpMode {

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
    public static int OBSERVATION_Y = 48 * bluef;
    public static int MID_X = -24 * bluef;
    public static int MID_Y = 48 * bluef;
    public Pose2d startPose;
    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDriveCancelable drive;
    private AprilReader reader;

    public static int SUBMERSIBLE_VEL = 30;

//    public void drive2distance(double target, double tolerance) {
//        updateRunningAverage();
//        double curDist = getRunningAverage();
//        while (Math.abs(curDist - target) > tolerance) {
//            updateRunningAverage();
//            curDist = getRunningAverage();
//            telemetry.addData("Current Distance", curDist);
//            dashTelemetry.addData("Current Distance", curDist);
//            telemetry.update();
//            dashTelemetry.update();
//            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
//                    .forward(curDist - target)
//                    .build();
//            drive.followTrajectorySequenceAsync(trajSeq);
//            startPose = trajSeq.end();
//            while (drive.isBusy()) {
//                drive.update();
//                if (isStopRequested()) {
//                    return;
//                }
//                updateRunningAverage();
//            }
//        }
//    }

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

    public void runApril() {
        if (doApril) {
            double[] april;
            long startTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - startTime < APRIL_TIME)) {
                bot.frames.loop();
                aprilLoop();
                if (isStopRequested()) {
                    return;
                }
            }
        }
    }

    public void aprilLoop() {
        if (doApril) {
            double[] april = reader.read();
            if (april.length == 4) {
                double x = april[0];
                double y = april[1];
                double yaw = april[2];
                int id = (int)(april[3]);
                startPose = new Pose2d(x, y, yaw);
                drive.setPoseEstimate(startPose);
                status("Pose Updated with April Tag #" + id);
            }
        }
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
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        drive.setPoseEstimate(startPose);

        // April Tags
        reader = new AprilReader(hardwareMap);

        // Wait for start
        while (!isStarted()) {
            if (isStopRequested()) {
                return;
            }
        }





        /*
        --- MOVE TO SUBMERSIBLE ---
        +20pts

        Sync:
        1. Drive to submersible
        2. Align with distance sensor
        3. Place specimen on high chamber

        Async:
        - Move left viper to placing position
        - Update distance sensor running average
         */

        bot.viper.move(3050, Viper.Sides.LEFT); // Start lifting viper
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(new MecanumVelocityConstraint(SUBMERSIBLE_VEL, DriveConstants.TRACK_WIDTH))
                .lineToConstantHeading(new Vector2d(SUBMERSIBLE_X, SUBMERSIBLE_Y))
                .resetVelConstraint()
                .build();
        status("Going to submersible...");
        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            drive.update();
            if (isStopRequested()) {
                return;
            }
            updateRunningAverage();
        }

        bot.frames.topPole();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            aprilLoop();
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
            aprilLoop();
        }





        /*
        --- MOVE TO OBSERVATION ZONE ---

        Sync:
        1. Drive to observation zone
        2. Align with distance sensor
        3. Pick up specimen from wall

        Async:
        - Lower left viper slide
        - Update distance sensor running average
         */

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .addDisplacementMarker(2, () -> {
                    bot.viper.move(WallPicker.wallPickerPos, Viper.Sides.LEFT);
                })
                .lineToLinearHeading(new Pose2d(OBSERVATION_X, OBSERVATION_Y, Math.toRadians(INVERTED_START_ANGLE)))
                .forward(4)
                .build();
        status("Moving to observation zone...");
        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            drive.update();
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
            aprilLoop();
        }





        /*
        --- MOVE TO SUBMERSIBLE ---
        +20pts

        Sync:
        1. Move to submersible
        2. Align with distance sensor
        3. Place specimen on high chamber

        Async
        - Move left viper to placing position
        - Update distance sensor running average
         */

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(4)
                .lineToLinearHeading(new Pose2d(SUBMERSIBLE_X2, SUBMERSIBLE_Y, Math.toRadians(START_ANGLE)))
                .addDisplacementMarker(2, () -> {
                    bot.viper.move(ChamberPlacer.startViper, Viper.Sides.LEFT);
                })
                .build();
        status("Going to submersible...");
        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            drive.update();
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
                .addDisplacementMarker(2, () -> {
                    bot.viper.move(0, Viper.Sides.LEFT);
                })
                .lineToConstantHeading(new Vector2d(OBSERVATION_X, OBSERVATION_Y))
                .back(4)
                .build();
        status("Moving to observation zone...");
        drive.followTrajectorySequence(trajSeq);
        startPose = trajSeq.end();

        bot.frames.zeroBasket();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }
    }
}
