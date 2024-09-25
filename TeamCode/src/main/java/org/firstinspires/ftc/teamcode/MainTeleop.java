package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Color Sensor Test", group = "Iterative Opmode")
public class MainTeleop extends OpMode {
    Hardware map;

    @Override
    public void init() {
        map = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {

        map.colorDistanceSensor.loop();

        telemetry.addData("Red: ", map.colorDistanceSensor.red);
        telemetry.addData("Red_Val: ", map.colorDistanceSensor.RED_VAL);
        telemetry.addData("Green: ", map.colorDistanceSensor.green);
        telemetry.addData("Green_Val: ", map.colorDistanceSensor.GREEN_VAL);
        telemetry.addData("Blue: ", map.colorDistanceSensor.blue);
        telemetry.addData("Blue_Val: ", map.colorDistanceSensor.BLUE_VAL);
        telemetry.addData("Color: ", map.colorDistanceSensor.color);
        telemetry.addData("Range", map.colorDistanceSensor.READING_DISTANCE);
        telemetry.update();
    }
}
