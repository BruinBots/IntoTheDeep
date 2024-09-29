package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Viper {
    public static int MAX_VIPER_POS = 10000;
    public static int MIN_VIPER_POS = 0;
    public static int VIPER_SPEED = 2;
    public static double VIPER_POWER = 0.4;

    private DcMotorEx motor;

    public Viper(DcMotorEx motor) {
        this.motor = motor;
    }

    public void move(int targetPos) {
        // if viper pos is greater or less than max/min then set to max/min
        if (targetPos < MIN_VIPER_POS) {
            targetPos = MIN_VIPER_POS;
        } else if (targetPos > MAX_VIPER_POS) {
            targetPos = MAX_VIPER_POS;
        }

        motor.setPower(VIPER_POWER);
        motor.setTargetPosition(targetPos);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
