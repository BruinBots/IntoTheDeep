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
import com.qualcomm.robotcore.hardware.DcMotor;

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
@Autonomous(name = "B-Comp (3) OneAscentAuto", preselectTeleOp = "Main Teleop")
public class OneAscentAuto extends LinearOpMode {

    public static int START_X = -36;
    public static int START_Y = -60;
    public static int START_ANGLE = 90;
    public static int ASCENT_ANGLE = -90;

    public Pose2d startPose;
    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDriveCancelable drive;
    private AutoDistance distDriver;

    public static int AX = -48;
    public static int AY = -10;

    public static int BX = -30;
    public static int BY = -6;

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

        startPose = distDriver.drive2distance(1, 0.5);
        if (startPose == null) {
            return;
        }

        bot.viperMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.viperMotorL.setPower(-0.3);

        long curTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - curTime < 1500) {
            if (isStopRequested()) {
                return;
            }
        }
    }
}
