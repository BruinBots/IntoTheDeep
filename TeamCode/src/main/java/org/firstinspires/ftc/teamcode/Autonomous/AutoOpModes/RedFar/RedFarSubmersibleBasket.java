package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.RedFar;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BaseAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueFarAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.RedFarAuto;

@Autonomous(name="Red Far Submersible Basket", group="Red Far")
public class RedFarSubmersibleBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RedFarAuto auto = new RedFarAuto(hardwareMap, telemetry, this);
        waitForStart();
        auto.basket(RedFarAuto.startPose);
        auto.run(BaseAuto.AutoOperation.SUBMERSIBLE, BaseAuto.AutoOperation.SAMPLES, BaseAuto.AutoOperation.BASKET);
    }
}
