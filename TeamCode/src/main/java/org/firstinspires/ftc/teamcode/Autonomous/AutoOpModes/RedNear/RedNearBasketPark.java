package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedNear;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueFarAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.RedFarAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.RedNearAuto;

@Autonomous(name="Red Near Basket Park", group="Red Near")
public class RedNearBasketPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RedNearAuto auto = new RedNearAuto(hardwareMap, telemetry);
        waitForStart();
        auto.park(auto.basket(RedNearAuto.startPose));
    }
}
