package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class MotorBurner {
    public DcMotorEx motor;
    public double maxCurrent;
    private OpMode opMode;
    private int gamepad;
    private String name;

    public static double MAX_CURRENT = 6;

    public MotorBurner(DcMotorEx motor, double maxCurrent, OpMode opMode, int gamepad, String name) {
        this.motor = motor;
        this.maxCurrent = maxCurrent;
        this.opMode = opMode;
        this.gamepad = gamepad;
        this.name = name;
    }

    public void loop() {
        MainTeleop.doTelemetry("MotorBurner " + name, motor.getCurrent(CurrentUnit.AMPS)+"A");
        if (motor.getCurrent(CurrentUnit.AMPS) > MAX_CURRENT) {
            if (gamepad == 0 || gamepad == 1) {
                opMode.gamepad1.rumble(1000);
            }
            if (gamepad == 0 || gamepad == 2) {
                opMode.gamepad2.rumble(1000);
            }
        }
    }
}
