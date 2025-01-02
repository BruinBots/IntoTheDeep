package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ViperOp", group = "Testing Opmode")
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
            bot.viper.move(0);
        }
        //Set slide to midway
        else if (gamepad1.b) {
            bot.viper.move(1600);
        }
        //Set slide to max
        else if (gamepad1.y) {
            bot.viper.move(4200);
        }
        //Increase or decrease slide position
        else if (gamepad1.right_bumper) {
            bot.viper.moveUp(Viper.Sides.BOTH);
        } else if (gamepad1.left_bumper) {
            bot.viper.moveDown(Viper.Sides.BOTH);
        }

        // Telemetry of slide position
//        telemetry.addData("Target Position", bot.viper.getTargetPos());
//        telemetry.addData("Actual Position", bot.viper.getActualPos());
        telemetry.update();

        bot.viper.move(pos);
    }
}
