package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.BlueFar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueFarAuto;

@Autonomous(name="Blue Far Basket Park", group="Blue Far")
public class BlueFarBasketPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BlueFarAuto auto = new BlueFarAuto(hardwareMap, telemetry);
        waitForStart();
        auto.park(auto.basket(BlueFarAuto.startPose));
    }
}
