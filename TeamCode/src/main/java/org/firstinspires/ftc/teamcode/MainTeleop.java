package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main TeleOp", group = "Iterative Opmode")
public class MainTeleop extends OpMode {
    Hardware map;

    // drive values
    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;

    int armPos = 0;
    int wristPos = 0;

    @Override
    public void init() {
        map = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {

        drive = gamepad1.left_stick_y - gamepad2.left_stick_y;
        strafe = gamepad2.left_stick_x - gamepad1.left_stick_x;
        turn= gamepad1.right_stick_x + gamepad2.right_stick_x;

        if (drive > 1) { drive = 1; }
        if (strafe > 1) { strafe = 1; }
        if (turn > 1) { turn = 1; }

        strafe = Math.copySign(Math.pow(strafe, 2), strafe);
        drive = Math.copySign(Math.pow(drive, 2), drive);
        turn = Math.copySign(Math.pow(turn, 2), turn);

        if (gamepad1.dpad_up) {
            map.arm.moveArm(armPos + 1);
        }
        else if (gamepad1.dpad_down) {
            map.arm.moveArm(armPos - 1);
        }

        if (gamepad1.dpad_right) {
            map.arm.moveWrist(wristPos + 1);
        }
        else if (gamepad1.dpad_left) {
            map.arm.moveWrist(wristPos - 1);
        }

        map.moveBotMecanum(drive, turn, strafe,  0.65); // actually move the robot


//        map.colorDistanceSensor.loop();
//
//        telemetry.addData("Red: ", map.colorDistanceSensor.red);
//        telemetry.addData("Red_Val: ", map.colorDistanceSensor.RED_VAL);
//        telemetry.addData("Green: ", map.colorDistanceSensor.green);
//        telemetry.addData("Green_Val: ", map.colorDistanceSensor.GREEN_VAL);
//        telemetry.addData("Blue: ", map.colorDistanceSensor.blue);
//        telemetry.addData("Blue_Val: ", map.colorDistanceSensor.BLUE_VAL);
//        telemetry.addData("Color: ", map.colorDistanceSensor.color);
//        telemetry.addData("Range", map.colorDistanceSensor.READING_DISTANCE);
        telemetry.update();
    }
}
