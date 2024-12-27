package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import static java.lang.Thread.sleep;

import android.content.res.AssetManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Arm;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.Viper;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

@Config
public class BaseAuto {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Hardware bot;
    public SampleMecanumDrive drive;

    public boolean blue;
    public boolean near;

    public static Pose2d startPose = new Pose2d(0, 0, 0);

    public static int START_X;
    public static int START_Y;
    public static int HIGHWAY_Y;
    public static int PARK_X;
    public static int PARK_Y;
    public static int BASKET_X;
    public static int BASKET_Y;
    public static int SAMPLES_X;
    public static int SAMPLES_Y;
    public static int SUBMERSIBLE_X;
    public static int SUBMERSIBLE_Y;

    public static double PARK_ANGLE;
    public static double BASKET_PREANGLE;
    public static double BASKET_ANGLE;
    public static double SAMPLES_ANGLE;
    public static double SUBMERSIBLE_ANGLE;

    public static boolean peripherals_allowed = true;

    private final OpMode mode;
    private Telemetry dashTelemetry;
    private FtcDashboard dash;

    public BaseAuto(HardwareMap hardwareMap, Telemetry telemetry, boolean blue, boolean near, OpMode mode) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        dash = FtcDashboard.getInstance();
        dashTelemetry = dash.getTelemetry();

        this.blue = blue;
        this.near = near;
        this.mode = mode;

        bot = new Hardware(hardwareMap);

        int x_factor = near ? 1 : -1;
        int y_factor = blue ? 1 : -1;

        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        try (InputStream input = assetManager.open("coordinates.properties")) {
            Properties prop = new Properties();
            prop.load(input);
            START_X = x_factor*Integer.parseInt(prop.getProperty(near ? "start.near.x" : "start.far.x"));
            START_Y = y_factor*Integer.parseInt(prop.getProperty("start.y"));
            if (blue) {
                START_X = -START_X;
            }
            HIGHWAY_Y = y_factor*Integer.parseInt(prop.getProperty("highway.y"));
            PARK_X = x_factor*Integer.parseInt(prop.getProperty("park.x"));
            PARK_Y = y_factor*Integer.parseInt(prop.getProperty("park.y"));
            BASKET_X = x_factor*Integer.parseInt(prop.getProperty("basket.x"));
            BASKET_Y = y_factor*Integer.parseInt(prop.getProperty("basket.y"));
            SAMPLES_X = x_factor*Integer.parseInt(prop.getProperty("samples.x"));
            SAMPLES_Y = y_factor*Integer.parseInt(prop.getProperty("samples.y"));
            SUBMERSIBLE_X = Integer.parseInt(prop.getProperty("submersible.x"));
            SUBMERSIBLE_Y = y_factor*Integer.parseInt(prop.getProperty("submersible.y"));

            PARK_ANGLE = Math.toRadians(conditionallyInvertAngle(Integer.parseInt(prop.getProperty("park.angle")), false));
            BASKET_ANGLE = Math.toRadians(conditionallyInvertAngle(Integer.parseInt(prop.getProperty("basket.angle")), !blue));
            SAMPLES_ANGLE = Math.toRadians(conditionallyInvertAngle(Integer.parseInt(prop.getProperty("samples.angle")), !blue));
            SUBMERSIBLE_ANGLE = Math.toRadians(conditionallyInvertAngle(Integer.parseInt(prop.getProperty("submersible.angle")), !blue));
        }
        catch (IOException ex) {
            ex.printStackTrace();
        }

        startPose = new Pose2d(START_X, START_Y, Math.toRadians(blue ? 270 : 90));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
    }

    public static Vector2d pose2vector(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    public void wait(double seconds) {
        try {
            sleep((long) (seconds*1000));
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    public void wait(int seconds) { wait((double)seconds); }

    public int invertAngle(int angle) { return (angle+180>=360)?(angle-180):(angle+180); }
    public int conditionallyInvertAngle(int angle, boolean invert) { return invert?invertAngle(angle):angle; }

    public double closestAngle(double targetAngle) {
        double angle = targetAngle - drive.getPoseEstimate().getHeading();

        if (angle > Math.PI) {
            return closestAngle(angle - 2*Math.PI);
        }
        else if (angle < -Math.PI) {
            return closestAngle(angle + 2*Math.PI);
        }
        return angle;
    }

    public boolean isClosestAngle(double targetAngle) {
        return (targetAngle - drive.getPoseEstimate().getHeading()) == closestAngle(targetAngle);
    }

    public double angleToCoords(double x2, double y2) {
        double x1 = drive.getPoseEstimate().getX();
        double y1 = drive.getPoseEstimate().getY();
        double dx = x2 - x1;
        double dy = y2 - y1;
        double tan = dy / dx;
        return Math.atan(tan);
    }

    public Pose2d turn(double targetAngle, Pose2d startPose) {
//       double deltaAngle = closestAngle(targetAngle);
//       return turnRelative(deltaAngle, startPose);
        if (isClosestAngle(targetAngle)) {
            drive.turn(targetAngle - drive.getPoseEstimate().getHeading());
            return new Pose2d(startPose.getX(), startPose.getY(), targetAngle);
        }
        double deltaAngle = closestAngle(targetAngle);
        drive.turn(deltaAngle);
        return startPose.plus(new Pose2d(0, 0, deltaAngle));
    }

    public Pose2d turnRelative(double deltaAngle, Pose2d startPose) {
        drive.turn(deltaAngle);
        return startPose.plus(new Pose2d(0, 0, deltaAngle));
    }

    public void displayMotorTelemetry(String caption, DcMotorEx motor) {
        doTelemetry(caption, motor.getCurrentPosition() + "=>" + motor.getTargetPosition());
    }

    public void doTelemetry(String caption, Object obj) {
        telemetry.addData(caption, obj);
        dashTelemetry.addData(caption, obj);
    }

    public void doFrames() {
        while (bot.frames.isBusy()) {
            bot.frames.loop();
            displayMotorTelemetry("Arm Motor", bot.armMotor);
            displayMotorTelemetry("Viper Motor L", bot.viperMotorL);
            displayMotorTelemetry("Viper Motor R", bot.viperMotorR);
            doTelemetry("Wrist", bot.wristServo.getPosition());
            telemetry.update();
            dashTelemetry.update();
        }
    }

    /// ----- SUPER MODE ----- ///

    public void superMode() {

    }
}
