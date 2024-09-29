package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ViperOp", group = "Testing Opmode")
public class ViperOp extends OpMode {
    Hardware bot;

    int pos = 0;

    @Override
    public void init() {
        bot = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            pos += Viper.VIPER_SPEED;
        }
        else if (gamepad1.left_bumper) {
            pos -= Viper.VIPER_SPEED;
        }

        if (pos > Viper.MAX_VIPER_POS) {
            pos = Viper.MAX_VIPER_POS;
        }
        else if (pos < Viper.MIN_VIPER_POS) {
            pos = Viper.MIN_VIPER_POS;
        }

        telemetry.addData("Viper Position", pos);
        telemetry.update();

        bot.viper.move(pos);
    }
}
