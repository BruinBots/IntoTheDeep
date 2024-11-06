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
    public static int HIGHWAY_Y;
    public static int PARK_X;
    public static int PARK_Y;
    public static int BASKET_X;
    public static int BASKET_Y;
    public static int SAMPLES_X;
    public static int SAMPLES_Y;
    public static int SUBMERSIBLE_X;
    public static int SUBMERSIBLE_Y;

    public RedNearAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, startPose, false, true);
    }
}
