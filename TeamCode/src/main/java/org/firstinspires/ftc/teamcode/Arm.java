package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    public static int MAX_ARM_POS = 1040;
    public static int MIN_ARM_POS = 0;
    public static double ARM_POWER = 0.3;
    public static int ARM_SPEED = 4;

    public static double MAX_WRIST_POS = 1;
    public static double MIN_WRIST_POS = 0.22;
    public static double WRIST_SPEED = 0.008;

    private int armPos;
    private double wristPos;

    private DcMotorEx armMotor;
    private Servo wristServo;

    public static double WRIST_ARM_CONSTANT = 0.0005;

    public Arm(DcMotorEx armMotor, Servo wristServo) {
        this.armMotor = armMotor;
        this.wristServo = wristServo;

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveArm(int targetPos) {
        armPos = targetPos;

        if (targetPos < MIN_ARM_POS) {
            targetPos = MIN_ARM_POS;
        } else if (targetPos > MAX_ARM_POS) {
            targetPos = MAX_ARM_POS;
        }

        armMotor.setPower(ARM_POWER);
        armMotor.setTargetPosition(targetPos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveWrist(double targetPos) { moveWrist(targetPos, true); }

    public void moveWrist(double targetPos, boolean setPos) {
        if (setPos) {
            wristPos = targetPos;
        }

        if (targetPos > MAX_WRIST_POS) {
            targetPos = MAX_WRIST_POS;
        }
        else if (targetPos < MIN_WRIST_POS) {
            targetPos = MIN_WRIST_POS;
        }

        wristServo.setPosition(targetPos);
    }

    public void syncWristToArm() {
        moveWrist(wristPos + armPos * WRIST_ARM_CONSTANT, false);
    }

    public void loop() {
        if (wristPos > Arm.MAX_WRIST_POS) {
            wristPos = Arm.MAX_WRIST_POS;
        }
        else if (wristPos < Arm.MIN_WRIST_POS) {
            wristPos = Arm.MIN_WRIST_POS;
        }

        if (armPos > Arm.MAX_ARM_POS) {
            armPos = Arm.MAX_ARM_POS;
        }
        else if (armPos < Arm.MIN_ARM_POS) {
            armPos = Arm.MIN_ARM_POS;
        }

        // vert pos is 155
        // move wrist to 0.95 at vert pos

        // pickup pos is 850
        // servo is 0.24

        // lift pos is 713
        // servp is 0
        // 
    }
}
