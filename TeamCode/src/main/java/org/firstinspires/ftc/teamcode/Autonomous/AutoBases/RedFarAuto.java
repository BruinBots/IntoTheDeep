package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class RedFarAuto extends BaseAuto {
    public RedFarAuto(HardwareMap hardwareMap, Telemetry telemetry, OpMode mode) {
        super(hardwareMap, telemetry, false, false, mode);
    }
}
