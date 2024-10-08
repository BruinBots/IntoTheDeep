package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Do Nothing", group = "Umm...Nothing")
public class DoNothing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Aren't you going to do anything for autonomous?");
        telemetry.addLine("Nope! Bye.");
        telemetry.update();
    }
}
