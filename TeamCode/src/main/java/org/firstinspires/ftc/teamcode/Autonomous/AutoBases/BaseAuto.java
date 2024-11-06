package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import android.content.res.AssetManager;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.Vector;

public class BaseAuto {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final Hardware bot;
    public SampleMecanumDrive drive;

    public boolean blue;
    public boolean far;

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

    public BaseAuto(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startingPosition, boolean blue, boolean near) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.blue = blue;
        this.far = far;

        bot = new Hardware(hardwareMap);

        int x_factor = blue ? 1 : -1;
        int y_factor = near ? 1 : -1;

        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        try (InputStream input = assetManager.open("coordinates.properties")) {
            Properties prop = new Properties();
            prop.load(input);
            START_X = x_factor*Integer.parseInt(prop.getProperty("start.x"));
            START_Y = y_factor*Integer.parseInt(prop.getProperty("start.y"));
            HIGHWAY_Y = y_factor*Integer.parseInt(prop.getProperty("highway.y"));
            PARK_X = Integer.parseInt(prop.getProperty("park.x"));
            PARK_Y = y_factor*Integer.parseInt(prop.getProperty("park.y"));
            BASKET_X = Integer.parseInt(prop.getProperty("basket.x"));
            BASKET_Y = y_factor*Integer.parseInt(prop.getProperty("basket.y"));
            SAMPLES_X = x_factor*Integer.parseInt(prop.getProperty("samples.x"));
            SAMPLES_Y = y_factor*Integer.parseInt(prop.getProperty("samples.y"));
            SUBMERSIBLE_X = Integer.parseInt(prop.getProperty("submersible.x"));
            SUBMERSIBLE_Y = y_factor*Integer.parseInt(prop.getProperty("submersible.y"));
        }
        catch (IOException ex) {
            ex.printStackTrace();
        }

        startPose = new Pose2d(START_X, START_Y, Math.toRadians(blue ? 270 : 90));

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startingPosition);
    }

    public static Vector2d pose2vector(Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    ///// ---------- RUN ---------- /////

    public static enum AutoOperation {
        PARK,
        BASKET,
        SAMPLES,
        SUBMERSIBLE
    }

    public void run(AutoOperation... ops) {
        Pose2d curPos = startPose;
        int count = ops.length;
        int i = 0;
        for (AutoOperation op: ops) {
            switch (op) {
                case PARK:
                    curPos = park(curPos);
                    break;
                case BASKET:
                    curPos = basket(curPos);
                    break;
                case SAMPLES:
                    curPos = samples(curPos);
                    break;
                case SUBMERSIBLE:
                    curPos = submersible(curPos);
                    break;
            }

            if (i < count - 1) {
                curPos = highway(curPos);
            }

            i ++;
        }
    }

    // ----- HIGHWAY ----- //
    public double get_x() { return drive.getPoseEstimate().getX(); }

    public Vector2d highway_v() { return new Vector2d(get_x(), HIGHWAY_Y); }

    public Pose2d highway(Trajectory startTraj) {
        return highway(startTraj.end());
    }

    public Pose2d highway(Pose2d startPose) {
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(highway_v())
                .build();
        drive.followTrajectory(traj);
        return traj.end();
    }

    // ----- PARKING ----- //
    public Vector2d park_v() {
        return new Vector2d(PARK_X, PARK_Y);
    }

    public Pose2d park(Trajectory startTraj) {
        return park(startTraj.end());
    }

    public Pose2d park(Pose2d startPose) {
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(park_v())
                .build();
        drive.followTrajectory(traj);
        return traj.end();
    }

    // ----- BASKET ----- //
    public Vector2d basket_v() { return new Vector2d(BASKET_X, BASKET_Y); }

    public Pose2d basket(Trajectory startTraj) {
        return basket(startTraj.end());
    }

    public Pose2d basket(Pose2d startPose) {
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(basket_v())
                .build();
        drive.followTrajectory(traj);

        // TODO: Place sample in basket

        return traj.end();
    }

    // ----- SAMPLES ----- //
    public Vector2d samples_v() { return new Vector2d(SAMPLES_X, SAMPLES_Y); }

    public Pose2d samples(Trajectory startTraj) { return samples(startTraj.end()); }

    public Pose2d samples(Pose2d startPose) {
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(samples_v())
                .build();
        drive.followTrajectory(traj);

        // TODO: Pick up sample

        return traj.end();
    }

    // ----- SUBMERSIBLE ----- //
    public Vector2d submersible_v() { return new Vector2d(SUBMERSIBLE_X, SUBMERSIBLE_Y); }

    public Pose2d submersible(Trajectory startTraj) { return submersible(startTraj.end()); }

    public Pose2d submersible(Pose2d startPose) {
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(submersible_v())
                .build();
        drive.followTrajectory(traj);

        // TODO: Place specimen on horizontal bar

        return traj.end();
    }
}
