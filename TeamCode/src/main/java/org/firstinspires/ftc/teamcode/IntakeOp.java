package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="IntakeOp", group = "Testing Opmode")
public class IntakeOp extends OpMode {
    Hardware bot;

    @Override
    public void init() {
        bot = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            bot.intake.intake();
        }
        else if (gamepad1.b) {
            bot.intake.outtake();
        }
        else {
            bot.intake.stop();
        }
    }
}
