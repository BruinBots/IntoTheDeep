package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import static java.lang.Thread.sleep;

import android.content.res.AssetManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import java.util.Vector;

@Config
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

    public static int BASKET_ANGLE;
    public static int SAMPLES_ANGLE;
    public static int SUBMERSIBLE_ANGLE;

    public static boolean peripherals_allowed = false;

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

            BASKET_ANGLE = conditionallyInvertAngle(Integer.parseInt(prop.getProperty("basket.angle")), !blue);
            SAMPLES_ANGLE = conditionallyInvertAngle(Integer.parseInt(prop.getProperty("samples.angle")), !blue);
            SUBMERSIBLE_ANGLE = conditionallyInvertAngle(Integer.parseInt(prop.getProperty("submersible.angle")), !blue);
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

    public Pose2d posePlusAngle(Trajectory traj, int angle) { return posePlusAngle(traj.end(), angle); }

    public Pose2d posePlusAngle(Pose2d pose, int angle) {
        return pose.plus(new Pose2d(0, 0, Math.toRadians(angle)));
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
        curPos = highway(curPos);
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
        drive.turn(BASKET_ANGLE);
        Trajectory traj = drive.trajectoryBuilder(posePlusAngle(startPose, BASKET_ANGLE))
                .lineToConstantHeading(basket_v())
                .build();
        drive.followTrajectory(traj);

        // TODO: Place sample in basket
        if (peripherals_allowed) {
            // Lift viper slides
            bot.viper.move(Viper.MAX_VIPER_POS);
            wait(0.5);
            // Drop pixel
            bot.basket.setOpen();
            wait(0.5);
            bot.basket.setClosed();
            wait(0.5);
            // Lower viper slides
            bot.viper.move(Viper.MIN_VIPER_POS);
        }

        return traj.end();
    }

    // ----- SAMPLES ----- //
    public Vector2d samples_v() { return new Vector2d(SAMPLES_X, SAMPLES_Y); }

    public Pose2d samples(Trajectory startTraj) { return samples(startTraj.end()); }

    public Pose2d samples(Pose2d startPose) {
        drive.turn(SAMPLES_ANGLE);
        Trajectory traj = drive.trajectoryBuilder(posePlusAngle(startPose, SAMPLES_ANGLE))
                .lineToConstantHeading(samples_v())
                .build();
        drive.followTrajectory(traj);

        // TODO: Pick up sample
        if (peripherals_allowed) {
            // Lower arm
            bot.arm.moveWrist(Arm.MAX_WRIST_POS);
            bot.arm.moveArm(Arm.MAX_ARM_POS);
            wait(0.5);
            // Pick up sample
            bot.intake.engage();
            wait(0.5);
            // Retract arm
            bot.arm.moveArm(Arm.MIN_ARM_POS);
            bot.arm.moveWrist(Arm.MIN_WRIST_POS);
            wait(0.5);
            // Lower viper
            bot.viper.move(Viper.MIN_VIPER_POS);
            bot.basket.setOpen();
            // Handoff
            bot.intake.standby();
            wait(1);
            bot.basket.setClosed();
        }

        return traj.end();
    }

    // ----- SUBMERSIBLE ----- //
    public Vector2d submersible_v() { return new Vector2d(SUBMERSIBLE_X, SUBMERSIBLE_Y); }

    public Pose2d submersible(Trajectory startTraj) { return submersible(startTraj.end()); }

    public Pose2d submersible(Pose2d startPose) {
        drive.turn(SUBMERSIBLE_ANGLE);
        Trajectory traj = drive.trajectoryBuilder(posePlusAngle(startPose, SUBMERSIBLE_ANGLE))
                .lineToConstantHeading(submersible_v())
                .build();
        drive.followTrajectory(traj);

        // TODO: Place specimen on horizontal bar
        if (peripherals_allowed) {
            // Lift viper
            bot.viper.move(Viper.MAX_VIPER_POS);
            wait(1);
            // Basket to middle position
            bot.basket.setMiddle();
            wait(0.5);
            // Slowly lower viper
            bot.viper.move(Viper.MAX_VIPER_POS-1000);
            wait(2);
            // Open basket
            bot.basket.setOpen();
            wait(0.5);
            // Lower viper
            bot.viper.move(Viper.MIN_VIPER_POS);
            wait(1);
        }
        return traj.end();
    }
}
