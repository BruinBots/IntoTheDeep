package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedFar;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.AprilReader;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Autonomous(name = "SuperAuto", preselectTeleOp = "SuperAuto")
public class SuperAuto extends LinearOpMode {

    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDrive drive;
    private AprilReader reader;

    public static boolean blue = false;
    private static int bluef = blue ? 1 : -1;

    public Pose2d startPose;

    public static int START_X = -10;
    public static int START_Y = 60*bluef;
    public static int START_ANGLE = blue ? 90 : 270;
    public static int INVERTED_START_ANGLE = blue ? 270 : 90;
    public static int SUBMERSIBLE_X = 0;
    public static int SUBMERSIBLE_Y = 32*bluef;

    public double turnRelative(double targetAngle) {
        double curAngle = drive.getPoseEstimate().getHeading();
        double deltaAngle = targetAngle - curAngle;
        double plusAngle = (targetAngle + Math.PI*2) - curAngle;
        double minusAngle = (targetAngle - Math.PI*2) - curAngle;
        return Math.min(Math.abs(deltaAngle), Math.min(Math.abs(plusAngle), Math.abs(minusAngle)));
    }

    public double turnToCoords(double x2, double y2) {
        double x1 = drive.getPoseEstimate().getX();
        double y1 = drive.getPoseEstimate().getY();
        double dx = x2 - x1;
        double dy = y2 - y1;
        double tan = dy / dx;
        return Math.atan2(dy, dx);
    }

    public Pose2d turn(double angle) {
        drive.turn(angle);
        return startPose.plus(new Pose2d(0, 0, angle));
    }

    @Override
    public void runOpMode() {
        bot = new Hardware(hardwareMap);

        // Dashboard telemetry
        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();

        // RR
        startPose = new Pose2d(START_X, START_Y, Math.toRadians(START_ANGLE));
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);

        // April Tags
        reader = new AprilReader(hardwareMap);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .back(6)
                .turn(turnRelative(turnToCoords(SUBMERSIBLE_X, SUBMERSIBLE_Y)))
                .lineToConstantHeading(new Vector2d(SUBMERSIBLE_X, SUBMERSIBLE_Y))
                .addDisplacementMarker(20, () -> {
                    // Start lifting viper
                })
                .turn(turnRelative(Math.toRadians(START_ANGLE)))
                .build();
        drive.followTrajectorySequence(trajSeq);
        startPose = trajSeq.end();

        // CHAMBER
        // bot.frames.specimenToHighChamber()
        while (bot.frames.isBusy() && !isStopRequested()) {
            bot.frames.loop();
        }

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(6)
                .addDisplacementMarker(() -> {
                    // Start lowering viper
                })
                .turn(turnRelative(turnToCoords(48, -56)))
                .lineToConstantHeading(new Vector2d(48, -56))
                .turn(turnRelative(Math.toRadians(INVERTED_START_ANGLE)))
                .back(6)
                .build();
        drive.followTrajectorySequence(trajSeq);
        startPose = trajSeq.end();

        // WALL
        // bot.frames.specimenFromWall()
        double[] april;
        long startTime = System.currentTimeMillis();
        while (bot.frames.isBusy() && System.currentTimeMillis() - startTime < 1000 && !isStopRequested()) {
            bot.frames.loop();
            april = reader.read();
            if (april.length == 2) {
                double x = april[0];
                double y = april[1];
                startPose = new Pose2d(x, y, drive.getPoseEstimate().getHeading());
                drive.setPoseEstimate(startPose);
                telemetry.addData("RR Status", "Pose Updated");
                telemetry.update();
            }
        }


    }
}
