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
        // Set slide to zero
        if (gamepad1.a) {
            pos = 0;
        }
        //Set slide to midway
        else if (gamepad1.b){
            pos = 1600;
        }
        //Set slide to max
        else if (gamepad1.y) {
            pos = 4200;
        }
        //Increase or decrease slide position
        else if (gamepad1.right_bumper) {
            pos += Viper.VIPER_SPEED;
        }
        else if (gamepad1.left_bumper) {
            pos -= Viper.VIPER_SPEED;
        }

        // Don't let the slide go past the max or min
        if (pos > Viper.MAX_VIPER_POS) {
            pos = Viper.MAX_VIPER_POS;
        }
        else if (pos < Viper.MIN_VIPER_POS) {
            pos = Viper.MIN_VIPER_POS;
        }

        // Telemetry of slide position
        telemetry.addData("Target Position", bot.viper.getTargetPos());
        telemetry.addData("Actual Position", bot.viper.getActualPos());
        telemetry.update();

        bot.viper.move(pos);
    }
}
