package org.firstinspires.ftc.teamcode.testantiexplosion;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Test EzraOp", group = "Iterative Opmode")
public class TeleOperatedMode extends OpMode {
    HwMap bot;

    public int motorPos = 0;
    private long stopTime = 0;
    private boolean isStopped = false;

    @Override
    public void init() {
        bot = new HwMap(hardwareMap);
        bot.motor.setTargetPosition(motorPos);
        bot.motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        bot.motor.setPower(1);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            motorPos += 5;
        }

        bot.motor.setTargetPosition(motorPos);

        checkMotorPosition();

        if (isStopped) {
            if (System.currentTimeMillis() - stopTime >= 100) {
                bot.motor.setPower(1);
                isStopped = false;
            }
        }

        telemetry.addData("Motor Velocity", bot.motor.getVelocity());
        telemetry.addData("Current Position", bot.motor.getCurrentPosition());
        telemetry.addData("Target Position", bot.motor.getTargetPosition());
        telemetry.addData("Motor amp", bot.motor.getCurrent(CurrentUnit.AMPS));
    }

    public void checkMotorPosition() {
        if (Math.abs(bot.motor.getVelocity()) <= 5 && bot.motor.getCurrent(CurrentUnit.AMPS) > 5) { // amps go up high when its stressing
            if (Math.abs(bot.motor.getCurrentPosition() - bot.motor.getTargetPosition()) > 10) {
                gamepad1.rumble(1000);
                motorPos = bot.motor.getCurrentPosition();
                bot.motor.setPower(0);
                stopTime = System.currentTimeMillis();
                isStopped = true;
            }
        }
    }
}