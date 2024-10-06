package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Arm {
    public static int MAX_ARM_POS = 1000;
    public static int MIN_ARM_POS = 0;
    public static double ARM_POWER = 0.4;
    public static int ARM_SPEED = 8;

    public static int MAX_WRIST_POS = 1;
    public static int MIN_WRIST_POS = 0;
    public static double WRIST_SPEED = 0.008;

    private DcMotorEx armMotor;
    private Servo wristServo;

    public Arm(DcMotorEx armMotor, Servo wristServo) {
        this.armMotor = armMotor;
        this.wristServo = wristServo;
    }

    public void moveArm(int targetPos) {
        if (targetPos < MIN_ARM_POS) {
            targetPos = MIN_ARM_POS;
        } else if (targetPos > MAX_ARM_POS) {
            targetPos = MAX_ARM_POS;
        }

        armMotor.setPower(ARM_POWER);
        armMotor.setTargetPosition(targetPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveWrist(double targetPos) {
        if (targetPos > MAX_WRIST_POS) {
            targetPos = MAX_WRIST_POS;
        }
        else if (targetPos < MIN_WRIST_POS) {
            targetPos = MIN_WRIST_POS;
        }

        wristServo.setPosition(targetPos);
    }
}
