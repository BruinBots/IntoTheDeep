package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedNear;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BaseAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueNearAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.RedNearAuto;

@Autonomous(name="Red Near Park", group="Red Near")
public class RedNearParkOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RedNearAuto auto = new RedNearAuto(hardwareMap, telemetry, this);
        waitForStart();
        auto.run(BaseAuto.AutoOperation.PARK);
    }
}
