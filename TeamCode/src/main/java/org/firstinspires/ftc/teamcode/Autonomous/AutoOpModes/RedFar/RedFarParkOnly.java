package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedFar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BaseAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueNearAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.RedFarAuto;

@Autonomous(name="Red Far Park", group="Red Far")
public class RedFarParkOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RedFarAuto auto = new RedFarAuto(hardwareMap, telemetry);
        waitForStart();
        auto.run(BaseAuto.AutoOperation.PARK);
    }
}
