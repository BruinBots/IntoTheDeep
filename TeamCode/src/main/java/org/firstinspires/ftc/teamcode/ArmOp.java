package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="ArmOp", group = "Testing Opmode")
public class ArmOp extends OpMode {
    Hardware bot;

    int armPos = 0;
    double wristPos = 0;

    @Override
    public void init() {
        bot = new Hardware(hardwareMap);
    }

    @Override
    public void loop() {
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

        telemetry.addData("Arm Position", armPos);
        telemetry.addData("Wrist Position", wristPos);
        telemetry.update();

        bot.arm.moveArm(armPos);
        bot.arm.moveWrist(wristPos);
    }
}
