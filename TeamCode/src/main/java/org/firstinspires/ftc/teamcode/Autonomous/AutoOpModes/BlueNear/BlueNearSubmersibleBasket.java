package org.firstinspires.ftc.teamcode.Autonomous.AutoOpModes.BlueNear;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BaseAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueFarAuto;
import org.firstinspires.ftc.teamcode.Autonomous.AutoBases.BlueNearAuto;

@Autonomous(name="Blue Near Submersible Basket", group="Blue Near")
public class BlueNearSubmersibleBasket extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BlueNearAuto auto = new BlueNearAuto(hardwareMap, telemetry, this);
        waitForStart();
        auto.run(BaseAuto.AutoOperation.SUBMERSIBLE, BaseAuto.AutoOperation.SAMPLES, BaseAuto.AutoOperation.BASKET);
    }
}
