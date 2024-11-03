package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="IntakeOp", group = "Testing Opmode")
public class IntakeOp extends OpMode {
    Hardware bot;

    //Instantiate variables
    boolean isPixel = false; // Is there a pixel in the robot?
    ColorDistanceSensor.Colors currentOP = (ColorDistanceSensor.Colors.red); // Exaple (red or blue)
    long outtakeTime = 200; // How long to outtake after getting a bad pixel (in ms)

    @Override
    public void init() {
        bot = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {
        // Telemetry
        telemetry.addData("Color", bot.ColorDistanceSensor.color);
        telemetry.addData("Distance", bot.ColorDistanceSensor.READING_DISTANCE);
        telemetry.addData("Is Pixel", isPixel);
        telemetry.update();

        // Get color and distance
        bot.ColorDistanceSensor.loop();

        // Check reading distance
        if (bot.ColorDistanceSensor.READING_DISTANCE <= 1) {
            isPixel = true;
        }
        else {
            isPixel = false;
        }

        // If there is a pixel, check if its red yellow or blue and manage it
        if (isPixel) {
            // If the pixel is not the current OP color, outtake for 1 second
            ColorDistanceSensor.Colors color = bot.ColorDistanceSensor.color;
            if (color != currentOP && color != ColorDistanceSensor.Colors.yellow) {
                bot.intake.outtakeForTime();
            } else {
                bot.intake.stop();
            }
        }
        // Let you control it manually
        if (gamepad1.a) {
            bot.intake.intake();
        } else if (gamepad1.b) {
            bot.intake.outtake();
        } else {
            bot.intake.stop();
        }
    }
}
