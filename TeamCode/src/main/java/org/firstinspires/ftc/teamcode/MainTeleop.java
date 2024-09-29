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

        map.moveBotMecanum(drive, turn, strafe,  0.65); // actually move the robot
    }
}
