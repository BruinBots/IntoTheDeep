package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Table of Contents", group = "Confused yet?")
public class DoNothing extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("._____.___...____.\n" +
                "|_..._/._.\\./.___|\n" +
                "..|.||.|.|.|.|....\n" +
                "..|.||.|_|.|.|___.\n" +
                "..|_|.\\___/.\\____|");
        telemetry.addLine("Naming Convention for Autonomous OpMode's:");
        telemetry.addLine("1. Location (Blue or Red) and (Near or Far)");
        telemetry.addLine("2. Operations (in order)");
        telemetry.addLine("Park - Self explanatory\nBasket - Place sample in basket\nSubmersible - Hang sample on submersible bar\nSamples - Pick up sample (automatically done if basket after submersible)");
        telemetry.addLine("3. Good luck!");
        telemetry.addLine("If this is too confusing, there's always TeleOp to make up for lost time...");
        telemetry.update();
        sleep(10000);
    }
}
