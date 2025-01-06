package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedFar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.AprilReader;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Viper;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Config
@Autonomous(name = "SuperAuto", preselectTeleOp = "Main Teleop")
public class SuperAuto extends LinearOpMode {

    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDrive drive;
    private AprilReader reader;

    public static boolean blue = false;
    private static int bluef = blue ? 1 : -1;

    public Pose2d startPose;

    public static int START_X = 10;
    public static int START_Y = 60*bluef;
    public static int START_ANGLE = blue ? 270 : 90;
    public static int INVERTED_START_ANGLE = blue ? 90 : 270;
    public static int SUBMERSIBLE_X = 0;
    public static int SUBMERSIBLE_Y = 42*bluef;
    public static int OBSERVATION_X = -45*bluef;
    public static int OBSERVATION_Y = 48*bluef;
    public static int MID_X = -24*bluef;
    public static int MID_Y = 48*bluef;
    public static int MID_ANGLE = START_ANGLE;
    public static int APRIL_TIME = 1000;
    public static boolean doApril = false;

    public void drive2distance(double target, double tolerance) {
        double sum = 0;
        for (int i = 0; i < 5; i ++) {
            sum += bot.DistanceSensor.getDistance(DistanceUnit.INCH);
        }
        double curDist = sum/5;

        while (Math.abs(curDist - target) > tolerance) {
            sum = 0;
            for (int i = 0; i < 5; i ++) {
                sum += bot.DistanceSensor.getDistance(DistanceUnit.INCH);
            }
            curDist = sum/5;
            telemetry.addData("Current Distance", curDist);
            dashTelemetry.addData("Current Distance", curDist);
            telemetry.update();
            dashTelemetry.update();
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .forward(curDist - target)
                    .build();
            drive.followTrajectorySequence(trajSeq);
            startPose = trajSeq.end();
        }
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
        waitForStart();

        // lift before start
        bot.frames.lift();
        while (bot.frames.isBusy() && !isStopRequested()) { bot.frames.loop(); }

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(6)
                .splineTo(new Vector2d(SUBMERSIBLE_X, SUBMERSIBLE_Y), Math.toRadians(START_ANGLE))
                .addDisplacementMarker(2, () -> {
                    // Start lifting viper
                    bot.viper.move(3050, Viper.Sides.LEFT);
                })
                .build();
        drive.followTrajectorySequence(trajSeq);
        startPose = trajSeq.end();

        bot.frames.topPole();
        while (bot.frames.isBusy() && !isStopRequested()) { bot.frames.loop(); }

        drive2distance(5.5, 0.25);

        bot.frames.topSpecimen();
        while (bot.frames.isBusy() && !isStopRequested()) { bot.frames.loop(); }

        while (bot.frames.isBusy() && !isStopRequested()) {
            bot.frames.loop();
        }

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .addDisplacementMarker(2, () -> {
                    bot.viper.move(180, Viper.Sides.LEFT);
                })
                .splineTo(new Vector2d(MID_X, MID_Y), Math.toRadians(MID_ANGLE))
                .build();
        drive.followTrajectorySequence(trajSeq);
        startPose = trajSeq.end();

        if (doApril) {
            double[] april;
            long startTime = System.currentTimeMillis();
            while ((System.currentTimeMillis() - startTime < APRIL_TIME) && !isStopRequested()) {
                bot.frames.loop();
                april = reader.read();
                if (april.length == 3) {
                    double x = april[0];
                    double y = april[1];
//                    double yaw = april[2];
                    double yaw = drive.getPoseEstimate().getHeading();
                    startPose = new Pose2d(x, y, yaw);
                    drive.setPoseEstimate(startPose);
                    telemetry.addData("RR Status", "Pose Updated");
                    telemetry.update();
                }
            }
        }

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(OBSERVATION_X, OBSERVATION_Y), Math.toRadians(INVERTED_START_ANGLE))
                .forward(4)
                .build();
        drive.followTrajectorySequence(trajSeq);
        startPose = trajSeq.end();

        drive2distance(4, 0.4);

        bot.basket.setClosed();
        bot.frames.afterWall();
        while (bot.frames.isBusy() && !isStopRequested()) {
            bot.frames.loop();
        }
        bot.frames.zeroArm();
        while (bot.frames.isBusy() && !isStopRequested()) {
            bot.frames.loop();
        }
    }
}
