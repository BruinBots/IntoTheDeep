package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.BlueFar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BaseAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueFarAuto;

@Autonomous(name="Blue Far Basket", group="Blue Far")
public class BlueFarBasketOnly extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BlueFarAuto auto = new BlueFarAuto(hardwareMap, telemetry, this);
        waitForStart();
        auto.run(BaseAuto.AutoOperation.BASKET);
    }
}
