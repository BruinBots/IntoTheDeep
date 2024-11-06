package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedFar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BaseAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueFarAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.RedFarAuto;

@Autonomous(name="Red Far Basket Samples Park", group="Red Far")
public class RedFarBasketSamplesPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RedFarAuto auto = new RedFarAuto(hardwareMap, telemetry);
        waitForStart();
        auto.basket(RedFarAuto.startPose);
        auto.run(BaseAuto.AutoOperation.BASKET, BaseAuto.AutoOperation.SAMPLES, BaseAuto.AutoOperation.PARK);
    }
}
