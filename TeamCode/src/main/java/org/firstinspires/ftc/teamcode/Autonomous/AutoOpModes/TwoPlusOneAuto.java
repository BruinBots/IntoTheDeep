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
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.AutoDistance;
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
@Autonomous(name = "Z-Comp (43) TwoPlusOneAuto", preselectTeleOp = "Main Teleop")
public class TwoPlusOneAuto extends LinearOpMode {

    public static boolean blue = false;
    private static int bluef = blue ? 1 : -1;

    public static int START_X = bluef*-10;
    public static int START_ANGLE = blue ? 270 : 90;
    public static int INVERTED_START_ANGLE = blue ? 90 : 270;
    public static int SUBMERSIBLE_X = 0;
    public static int SUBMERSIBLE_X2 = bluef*-4;
    public static int START_Y = 60 * bluef;
    public static int SUBMERSIBLE_Y = 30 * bluef;
    public static int OBSERVATION_X = -45 * bluef;
    public static int OBSERVATION_Y = 48 * bluef;

    public Pose2d startPose;
    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDriveCancelable drive;
    private AprilReader reader;
    private AutoDistance distDriver;

    public static int SUBMERSIBLE_VEL = 40;

    public static int SAMPLE0X = bluef*-36;
    public static int SAMPLE_PUSH = 48;

    public static int STRAFE_DIST = 10;
    public static int PRE_SAMPLE_BACK_DIST = 24;

    public static int OBSERVATION_BACK_DIST = 14;


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

        distDriver = new AutoDistance(bot, this, drive, telemetry, dashTelemetry);

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
            distDriver.updateRunningAverage();
        }

        bot.frames.topPole();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }

        status("Aligning with distance sensor...");
        startPose = distDriver.drive2distance(ChamberPlacer.chamberPlacerDistance, ChamberPlacer.chamberPlacerTolerance);
        if (startPose == null) {
            return;
        }

        status("Placing specimen on high chamber...");
        bot.frames.topSpecimen();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
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
            distDriver.updateRunningAverage();
        }

        status("Aligning with distance sensor...");
        startPose = distDriver.drive2distance(WallPicker.wallPickerDistance, WallPicker.wallPickerTolerance);
        if (startPose == null) {
            return;
        }

        status("Picking up specimen...");
        bot.basket.setClosed();
        bot.frames.afterWall();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
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
            distDriver.updateRunningAverage();
            if (isStopRequested()) {
                return;
            }
        }

        status("Aligning with distance sensor...");
        startPose = distDriver.drive2distance(ChamberPlacer.chamberPlacerDistance, ChamberPlacer.chamberPlacerTolerance);
        if (startPose == null) {
            return;
        }

        status("Placing specimen on high chamber...");
        bot.frames.topSpecimen();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(12)
                .addDisplacementMarker(() -> {
                    bot.viper.move(0, Viper.Sides.LEFT);
                })
                .lineToLinearHeading(new Pose2d(SAMPLE0X, drive.getPoseEstimate().getY(), Math.toRadians(INVERTED_START_ANGLE)))
                .back(PRE_SAMPLE_BACK_DIST)
                .strafeLeft(STRAFE_DIST)
                .forward(SAMPLE_PUSH)
                .addDisplacementMarker(2, () -> {
                    bot.viper.move(WallPicker.wallPickerPos, Viper.Sides.LEFT);
                })
                .back(OBSERVATION_BACK_DIST)
                .forward(OBSERVATION_BACK_DIST)
                .build();

        status("Pushing sample...");
        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            drive.update();
            if (isStopRequested()) {
                return;
            }
        }

        bot.frames.zeroBasket();
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            if (isStopRequested()) {
                return;
            }
        }
    }
}
