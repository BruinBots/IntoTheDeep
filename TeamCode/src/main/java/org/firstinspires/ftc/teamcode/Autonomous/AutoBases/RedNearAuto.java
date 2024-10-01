package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import android.content.res.AssetManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.Vector;

@Config
public class RedNearAuto extends BaseAuto {
    public static Pose2d startPose = new Pose2d(0, 0, 0);

    public static int START_X;
    public static int START_Y;
    public static int PARK_X;
    public static int PARK_Y;

    public RedNearAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, startPose, true);

        AssetManager assetManager = AppUtil.getDefContext().getAssets();
        try (InputStream input = assetManager.open("coordinates.properties")) {
            Properties prop = new Properties();
            prop.load(input);
            START_X = Integer.parseInt(prop.getProperty("start.x"));
            START_Y = -Integer.parseInt(prop.getProperty("start.y"));
            PARK_X = -Integer.parseInt(prop.getProperty("park.x"));
            PARK_Y = -Integer.parseInt(prop.getProperty("park.y"));
        }
        catch (IOException ex) {
            ex.printStackTrace();
        }

        startPose = new Pose2d(START_X, START_Y, Math.toRadians(270));
    }

    @Override
    public Vector2d park_v() {
        return new Vector2d(PARK_X, PARK_Y);
    }
}
