package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import android.content.res.AssetManager;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;
import java.util.Vector;

@Config
public class RedFarAuto extends BaseAuto {
    public RedFarAuto(HardwareMap hardwareMap, Telemetry telemetry, OpMode mode) {
        super(hardwareMap, telemetry, startPose, false, false, mode);
    }
}
