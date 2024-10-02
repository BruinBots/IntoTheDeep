package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

@Config
public class Arm {
    public static int MAX_ARM_POS = 100;
    public static int MIN_ARM_POS = 0;
    public static int ARM_TOLERANCE = 5;
    public static double ARM_SPEED = 0.5;

    public static int MAX_WRIST_POS = 100;
    public static int MIN_WRIST_POS = 0;
    public static int WRIST_TOLERANCE = 2;
    public static double WRIST_SPEED = 0.2;

    private Servo armServo;
    private Servo wristServo;
    private Encoder armEncoder;
    private Encoder wristEncoder;

    public Arm(Servo armServo, Servo wristServo, Encoder armEncoder, Encoder wristEncoder) {
        this.armServo = armServo;
        this.wristServo = wristServo;
        this.armEncoder = armEncoder;
        this.wristEncoder = wristEncoder;
    }

    public void stop() {
        armServo.setPosition(0.5);
        wristServo.setPosition(0.5);
    }

    public void moveArm(double targetPos) {
        if (targetPos > MAX_ARM_POS) {
            targetPos = MAX_ARM_POS;
        }
        else if (targetPos < MIN_ARM_POS) {
            targetPos = MIN_ARM_POS;
        }

        int currentPos = armEncoder.getCurrentPosition();
        int currentMin = currentPos - ARM_TOLERANCE;
        int currentMax = currentPos - ARM_TOLERANCE;

        if (targetPos > currentMax) {
            armServo.setPosition(0.5 - ARM_SPEED/2);
        }
        else if (targetPos < currentMin) {
            armServo.setPosition(0.5 + ARM_SPEED/2);
        }
        else {
            armServo.setPosition(0.5);
        }
    }

    public void moveWrist(double targetPos) {
        if (targetPos > MAX_WRIST_POS) {
            targetPos = MAX_WRIST_POS;
        }
        else if (targetPos < MIN_WRIST_POS) {
            targetPos = MIN_WRIST_POS;
        }

        int currentPos = wristEncoder.getCurrentPosition();
        int currentMin = currentPos - WRIST_TOLERANCE;
        int currentMax = currentPos - WRIST_TOLERANCE;

        if (targetPos > currentMax) {
            wristServo.setPosition(0.5 - WRIST_SPEED/2);
        }
        else if (targetPos < currentMin) {
            wristServo.setPosition(0.5 + WRIST_SPEED/2);
        }
        else {
            wristServo.setPosition(0.5);
        }
    }
}
