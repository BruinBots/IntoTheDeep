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
@Autonomous(name = "B-Comp (19) SampleAscentAuto", preselectTeleOp = "Main Teleop")
public class SampleAscentAuto extends LinearOpMode {

    public static int START_X = -36;
    public static int START_Y = -60;
    public static int START_ANGLE = 90;
    public static int MID_ANGLE = START_ANGLE;
    public static int INVERTED_START_ANGLE = 270;
    public static int ASCENT_ANGLE = 0;
    public static int BASKET_ANGLE = 225;

    public Pose2d startPose;
    private Hardware bot;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;
    private SampleMecanumDriveCancelable drive;
    private AutoDistance distDriver;

    public static int AX = -54;
    public static int AY = -50;

    public static int BX = -48;
    public static int BY = -10;

    public static int CX = -30;
    public static int CY = -6;

    public static int VIPER = 6050;

    public static double BASKET_DISTANCE = 8.5;
    public static double ASCENT_DISTANCE = 1;

    public static int VEL = 20;

    public static double POWER = 0.3;

    public void tele(String caption, String message) {
        telemetry.addData(caption, message);
        dashTelemetry.addData(caption, message);
        telemetry.update();
        dashTelemetry.update();
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

        waitForStart();

        bot.viper.move(VIPER, Viper.Sides.LEFT);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .setVelConstraint(new MecanumVelocityConstraint(VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(AX, AY, Math.toRadians(BASKET_ANGLE)))
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            drive.update();
            if (isStopRequested()) {
                return;
            }
        }

        startPose = distDriver.drive2distance(BASKET_DISTANCE, 0.5);
        if (startPose == null) {
            return;
        }

        bot.basket.setOpen();

        trajSeq = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(0.5)
                .back(6)
                .lineToConstantHeading(new Vector2d(BX, BY))
                .addDisplacementMarker(20, () -> {
                    bot.viper.move(ChamberPlacer.downViper, Viper.Sides.LEFT);
                })
//                .turn(Math.toRadians(ASCENT_ANGLE))
                .lineToLinearHeading(new Pose2d(CX, CY, Math.toRadians(ASCENT_ANGLE)))
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);
        startPose = trajSeq.end();

        while (drive.isBusy()) {
            drive.update();
            if (isStopRequested()) {
                return;
            }
        }

        startPose = distDriver.drive2distance(ASCENT_DISTANCE, 0.5);
        if (startPose == null) {
            return;
        }

        bot.viperMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bot.viperMotorL.setPower(-POWER);

        long curTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - curTime < 1500) {
            if (isStopRequested()) {
                return;
            }
        }
    }
}
