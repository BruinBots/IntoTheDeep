package org.firstinspires.ftc.teamcode.Autonomous.AutoBases;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class BlueFarAuto extends BaseAuto {
    public BlueFarAuto(HardwareMap hardwareMap, Telemetry telemetry, OpMode mode) {
        super(hardwareMap, telemetry, startPose, true, false, mode);
    }
}
