package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Viper {
    public static int MAX_VIPER_POS = 6100; // changed from 312rpm motor to 223rpm and adjusted num accordingly
    public static int MIN_VIPER_POS = 0;
    public static int VIPER_SPEED = 30;
    public static double VIPER_POWER = 0.75;

    private DcMotorEx motorL;
    private DcMotorEx motorR;

    public enum Position {
        LEFT,
        RIGHT,
        BOTH
    }

    public Viper(DcMotorEx motorL, DcMotorEx motorR) {
        this.motorL = motorL;
        this.motorR = motorR;
    }

    public void move(int targetPos) {
        move(targetPos, Position.BOTH);
    }

    public void move(int targetPos, Position side) {
        // if viper pos is greater or less than max/min then set to max/min
        if (targetPos < MIN_VIPER_POS) {
            targetPos = MIN_VIPER_POS;
        } else if (targetPos > MAX_VIPER_POS) {
            targetPos = MAX_VIPER_POS;
        }

        // set power of motors
        if (side == Position.LEFT || side == Position.BOTH) {
            motorL.setPower(VIPER_POWER);
        }
        if (side == Position.RIGHT || side == Position.BOTH) {
            motorR.setPower(VIPER_POWER);
        }

        // set position of motors
        if (side == Position.LEFT || side == Position.BOTH) {
            motorL.setTargetPosition(targetPos);
        }
        if (side == Position.RIGHT || side == Position.BOTH) {
            motorR.setTargetPosition(-targetPos);
        }

        // make motors run to position
        if (side == Position.LEFT || side == Position.BOTH) {
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (side == Position.RIGHT || side == Position.BOTH) {
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public int getTargetPos() {
        return motorL.getTargetPosition();
    }

    public int getActualPos() {
        return motorL.getCurrentPosition();
    }

    public void loop() {
        if (getTargetPos() > Viper.MAX_VIPER_POS) {
            move(Viper.MAX_VIPER_POS);
        } else if (getTargetPos() < Viper.MIN_VIPER_POS) {
            move(Viper.MIN_VIPER_POS);
        }
    }
}