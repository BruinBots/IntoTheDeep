package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Viper {
    public static int MAX_VIPER_POS = 6100; // changed from 312rpm motor to 223rpm and adjusted num accordingly
    public static int MIN_VIPER_POS = 0;
    public static int VIPER_SPEED = 30;
    public static double VIPER_POWER = 1;

    private DcMotorEx motorL;
    private DcMotorEx motorR;

    public enum Sides {
        LEFT,
        RIGHT,
        BOTH
    }

    public Viper(DcMotorEx motorL, DcMotorEx motorR) {
        this.motorL = motorL;
        this.motorR = motorR;
    }

    public void moveUp(Sides side){
        move(getTargetPos(side) + VIPER_SPEED, side);
    }

    public void moveDown(Sides side){
        move(getTargetPos(side) - VIPER_SPEED, side);
    }

    public void move(int targetPos) {
        move(targetPos, Sides.BOTH);
    }

    public void move(int targetPos, Sides side) {
        // if viper pos is greater or less than max/min then set to max/min
        if (targetPos < MIN_VIPER_POS) {
            targetPos = MIN_VIPER_POS;
        } else if (targetPos > MAX_VIPER_POS) {
            targetPos = MAX_VIPER_POS;
        }

        // set power of motors
        if (side == Sides.LEFT || side == Sides.BOTH) {
            motorL.setPower(VIPER_POWER);
        }
        if (side == Sides.RIGHT || side == Sides.BOTH) {
            motorR.setPower(VIPER_POWER);
        }

        // set position of motors
        if (side == Sides.LEFT || side == Sides.BOTH) {
            motorL.setTargetPosition(targetPos);
        }
        if (side == Sides.RIGHT || side == Sides.BOTH) {
            motorR.setTargetPosition(-targetPos);
        }

        // make motors run to position
        if (side == Sides.LEFT || side == Sides.BOTH) {
            motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (side == Sides.RIGHT || side == Sides.BOTH) {
            motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public int getTargetPos(Sides side) {
        if (side == Sides.LEFT) {
            return motorL.getTargetPosition();
        } else if (side == Sides.RIGHT) {
            return motorR.getTargetPosition();
        } else {
            return (motorL.getTargetPosition());
        }
    }

    public int getActualPos(Sides side) {
        if (side == Sides.LEFT) {
            return motorL.getCurrentPosition();
        } else if (side == Sides.RIGHT) {
            return motorR.getCurrentPosition();
        } else {
            return (motorL.getCurrentPosition());
        }
    }

    public void resetEncoders(){
        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        move(0);
    }

//    public void loop() {
//        if (getTargetPos() > Viper.MAX_VIPER_POS) {
//            move(Viper.MAX_VIPER_POS);
//        } else if (getTargetPos() < Viper.MIN_VIPER_POS) {
//            move(Viper.MIN_VIPER_POS);
//        }
//    }
}