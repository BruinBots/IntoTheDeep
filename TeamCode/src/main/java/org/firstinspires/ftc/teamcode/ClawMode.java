package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ClawMode")
public class ClawMode extends OpMode {
    public ClawMap claw1;
    public ClawMap claw2;

    @Override
    public void init() {
        claw1 = new ClawMap(hardwareMap, telemetry, gamepad1, "1");
        claw2 = new ClawMap(hardwareMap, telemetry, gamepad2, "2");

        claw1.turretFactor = 1;
        claw1.wristFactor = -1;

        claw2.turretFactor = 1;
        claw2.wristFactor = -1;

        claw1.init();
        claw2.init();
        claw1.moveWrist(1);
        claw2.moveWrist(1);
    }

    @Override
    public void loop() {
        claw1.loop();
        claw2.loop();
        telemetry.update();
    }
}
