package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Main TeleOp", group = "Iterative Opmode")
public class MainTeleop extends OpMode {
    Hardware bot;

    // drive values
    double drive = 0.0;
    double turn = 0.0;
    double strafe = 0.0;

    int viperPos = 0;
    int armPos = 0;
    double wristPos = 0;

    @Override
    public void init() {
        bot = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {

        // Drive
        drive = gamepad1.left_stick_y - gamepad2.left_stick_y;
        strafe = gamepad2.left_stick_x - gamepad1.left_stick_x;
        turn= gamepad1.right_stick_x + gamepad2.right_stick_x;

        if (drive > 1) { drive = 1; }
        if (strafe > 1) { strafe = 1; }
        if (turn > 1) { turn = 1; }

        strafe = Math.copySign(Math.pow(strafe, 2), strafe);
        drive = Math.copySign(Math.pow(drive, 2), drive);
        turn = Math.copySign(Math.pow(turn, 2), turn);

        // Arm
        if (gamepad1.dpad_up) {
            armPos += Arm.ARM_SPEED;
        }
        else if (gamepad1.dpad_down) {
            armPos -= Arm.ARM_SPEED;
        }

        if (armPos > Arm.MAX_ARM_POS) {
            armPos = Arm.MAX_ARM_POS;
        }
        else if (armPos < Arm.MIN_ARM_POS) {
            armPos = Arm.MIN_ARM_POS;
        }

        // Wrist
        if (gamepad1.dpad_right) {
            wristPos += Arm.WRIST_SPEED;
        }
        else if (gamepad1.dpad_left) {
            wristPos -= Arm.WRIST_SPEED;
        }

        if (wristPos > Arm.MAX_WRIST_POS) {
            wristPos = Arm.MAX_WRIST_POS;
        }
        else if (wristPos < Arm.MIN_WRIST_POS) {
            wristPos = Arm.MIN_WRIST_POS;
        }

        // Viper
        if (gamepad1.right_bumper) {
            viperPos += Viper.VIPER_SPEED;
        }
        else if (gamepad1.left_bumper) {
            viperPos -= Viper.VIPER_SPEED;
        }

        if (viperPos > Viper.MAX_VIPER_POS) {
            viperPos = Viper.MAX_VIPER_POS;
        }
        else if (viperPos < Viper.MIN_VIPER_POS) {
            viperPos = Viper.MIN_VIPER_POS;
        }

        // Telemetry
        telemetry.addData("Viper Position", viperPos);
        telemetry.addData("Arm Position", armPos);
        telemetry.addData("Wrist Position", wristPos);
        telemetry.update();

        bot.arm.moveArm(armPos); // Move arm
        bot.arm.moveWrist(wristPos); // Move wrists
        bot.viper.move(viperPos); // Move viper
        bot.moveBotMecanum(drive, turn, strafe,  0.65); // actually move the robot
    }
}
