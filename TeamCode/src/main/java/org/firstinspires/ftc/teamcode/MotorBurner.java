package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorBurner {
    public DcMotorEx motor;
    public double maxCurrent;
    private OpMode opMode;
    private int gamepad;

    public MotorBurner(DcMotorEx motor, double maxCurrent, OpMode opMode, int gamepad) {
        this.motor = motor;
        this.maxCurrent = maxCurrent;
        this.opMode = opMode;
    }

    public void loop() {
        if (motor.getCurrent(CurrentUnit.AMPS) > maxCurrent) {
            if (gamepad == 0 || gamepad == 1) {
                opMode.gamepad1.rumble(1000);
            }
            if (gamepad == 0 || gamepad == 2) {
                opMode.gamepad2.rumble(1000);
            }
        }
    }
}
