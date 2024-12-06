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

    public static boolean peripherals_allowed = false;

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

    ///// ---------- RUN ---------- /////

    public enum AutoOperation {
        PARK,
        BASKET,
        SAMPLES,
        SUBMERSIBLE
    }

    public void run(AutoOperation... ops) {
        bot.init(false);
        bot.basket.setClosed();
        bot.frames.lift();
        doFrames();

        Pose2d curPos = startPose;
        int count = ops.length;
        int i = 0;
        if (ops.length > 1) { //  && ops[0] != AutoOperation.PARK
            curPos = highway(curPos);
        }
        for (AutoOperation op: ops) {
            if (op == AutoOperation.PARK) {
                curPos = park(curPos);
            }
            else if (op == AutoOperation.BASKET) {
                curPos = basket(curPos);
            }
            else if (op == AutoOperation.SAMPLES) {
                curPos = samples(curPos);
            }
            else if (op == AutoOperation.SUBMERSIBLE) {
                curPos = submersible(curPos);
            }

//            while (drive.isBusy()) {
//                if (mode.gamepad1.a || mode.gamepad1.b || mode.gamepad1.x || mode.gamepad1.y) {
//                    drive.breakFollowing();
//                }
//            }

            if (i < count - 1) {
                curPos = highway(curPos);
            }

            i ++;
        }

        bot.frames.zeroArm();
        doFrames();
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

    // ----- HIGHWAY ----- //
    public double get_x() { return drive.getPoseEstimate().getX(); }

    public Vector2d highway_v() { return new Vector2d(get_x(), HIGHWAY_Y); }

    public Pose2d highway(Trajectory startTraj) {
        return highway(startTraj.end());
    }

    public Pose2d highway(Pose2d startPose) {
        if (drive.getPoseEstimate().getY() == HIGHWAY_Y) {
            Trajectory traj = drive.trajectoryBuilder(startPose)
                    .lineToConstantHeading(highway_v())
                    .build();
            drive.followTrajectory(traj);
            return traj.end();
        }
        return startPose;
    }

    // ----- PARKING ----- //
    public Vector2d park_v() {
        return new Vector2d(PARK_X, PARK_Y);
    }

    public Pose2d park(Trajectory startTraj) {
        return park(startTraj.end());
    }

    public Pose2d park(Pose2d startPose) {
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(6)
                .build();
        drive.followTrajectory(traj1);
        startPose = traj1.end();
        startPose = turn(PARK_ANGLE, startPose);
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(park_v())
                .build();
        drive.followTrajectory(traj2);
        return traj2.end();
    }

    // ----- BASKET ----- //
    public Vector2d basket_v() { return new Vector2d(BASKET_X, BASKET_Y); }

    public Pose2d basket(Trajectory startTraj) {
        return basket(startTraj.end());
    }

    public Pose2d basket(Pose2d startPose) {
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(6)
                .build();
        drive.followTrajectory(traj1);
        startPose = traj1.end();
        startPose = turn(angleToCoords(BASKET_X, BASKET_Y), startPose);
        Trajectory traj2 = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(basket_v())
                .build();
        drive.followTrajectory(traj2);
        startPose = traj2.end();
        startPose = turn(BASKET_ANGLE, startPose);

//        Trajectory traj3 = drive.trajectoryBuilder(startPose)
//                .forward(0)
//                .build();
//        drive.followTrajectory(traj3);
//        startPose = traj3.end();

        bot.frames.topBasket();
        doFrames(); // Wait to deposit in basket
        bot.frames.wait(500);
        doFrames();
        bot.basket.setClosed(); // Close basket
        bot.frames.zeroBasket(); // Lower viper slides
        doFrames(); // Wait to lower viper

        return startPose;
    }

    // ----- SAMPLES ----- //
    public Vector2d samples_v() { return new Vector2d(SAMPLES_X, SAMPLES_Y); }

    public Pose2d samples(Trajectory startTraj) { return samples(startTraj.end()); }

    public Pose2d samples(Pose2d startPose) {
        startPose = turn(SAMPLES_ANGLE, startPose);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(samples_v())
                .addDisplacementMarker(() -> {
                    if (peripherals_allowed) {
                        // Lower arm
                        bot.frames.beforeGrab();
                    }
                })
                .addTemporalMarker(0.5, () -> {
                    if (peripherals_allowed) {
                        bot.frames.afterGrab();
                    }
                })
//                        // Pick up sample
//                        bot.intake.engage();
//                        wait(0.5);
//                        // Retract arm
//                        bot.arm.moveArm(Arm.MIN_ARM_POS);
//                        bot.arm.moveWrist(Arm.MIN_WRIST_POS);
//                        wait(0.5);
//                        // Lower viper
//                        bot.viper.move(Viper.MIN_VIPER_POS);
//                        bot.basket.setOpen();
//                        // Handoff
//                        bot.intake.standby();
//                        wait(1);
//                        bot.basket.setClosed();
//                    }
//                })
                .build();
        drive.followTrajectory(traj);

        // TODO: Pick up sample


        return traj.end();
    }

    // ----- SUBMERSIBLE ----- //
    public Vector2d submersible_v() { return new Vector2d(SUBMERSIBLE_X, SUBMERSIBLE_Y); }

    public Pose2d submersible(Trajectory startTraj) { return submersible(startTraj.end()); }

    public Pose2d submersible(Pose2d startPose) {
        startPose = turn(SUBMERSIBLE_ANGLE, startPose);
        Trajectory traj = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(submersible_v())
                .addDisplacementMarker(() -> {
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
                })
                .build();
        drive.followTrajectory(traj);

        // TODO: Place specimen on horizontal bar

        return traj.end();
    }
}
