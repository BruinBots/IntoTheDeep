package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class Viper {
    public static int MAX_VIPER_POS = 4400;
    public static int MIN_VIPER_POS = 0;
    public static int VIPER_SPEED = 30;
    public static double VIPER_POWER = 0.75;

    private DcMotorEx motorL;
    private DcMotorEx motorR;

    public Viper(DcMotorEx motorL, DcMotorEx motorR) {
        this.motorL = motorL;
        this.motorR = motorR;
    }

    public void move(int targetPos) {
        // if viper pos is greater or less than max/min then set to max/min
        if (targetPos < MIN_VIPER_POS) {
            targetPos = MIN_VIPER_POS;
        } else if (targetPos > MAX_VIPER_POS) {
            targetPos = MAX_VIPER_POS;
        }

        // Set motor powers
        motorL.setPower(VIPER_POWER);
        motorR.setPower(VIPER_POWER);

        // Set positions reversed for each motor
        motorL.setTargetPosition(targetPos);
        motorR.setTargetPosition(-targetPos);

        // Activate motors to use encoders
        motorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getTargetPos() {
        return motorL.getTargetPosition();
    }
    public int getActualPos() {
        return motorL.getCurrentPosition();
    }
}
