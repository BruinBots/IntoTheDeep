package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Arm {
    public static int MAX_ARM_POS = 8340;
    public static int MIN_ARM_POS = 0;
    public static double ARM_POWER = 1;
    public static int ARM_SPEED = 50;

    private class Frame {
        int arm;
        double wrist;

        public Frame(int arm, double wrist) {
            this.arm = arm;
            this.wrist = wrist;
        }

        public void run() {
            moveArm(arm);
            moveWrist(wrist);
        }
    }

    /*
    Arm -> Basket handoff positions:
    Arm     Wrist
    949     0.224 <== GROUND
    405     0.224
    405     0.776
    377     0.776
    377     0.808
    338     0.808
    305     0.808 <== END
     */

    private Frame[] groundToRamp;

    public static double MAX_WRIST_POS = 1;
    public static double MIN_WRIST_POS = 0;
    public static double WRIST_SPEED = 0.008;

    private int armPos;
    private double wristPos;

    private DcMotorEx armMotor;
    private Servo wristServo;

    public static double WRIST_ARM_CONSTANT = 0.0005;

    public Arm(DcMotorEx armMotor, Servo wristServo) {
        this.armMotor = armMotor;
        this.wristServo = wristServo;

        this.groundToRamp = new Frame[]{
            new Frame(949, 0.224),
            new Frame(405, 0.224),
            new Frame(405, 0.776),
            new Frame(377, 0.776),
            new Frame(377, 0.808),
            new Frame(338, 0.808),
            new Frame(305, 0.808),
        };

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveArmToRamp() {
        for (Frame f: groundToRamp) {
            f.run();
            try {
                sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
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
        if (armMotor.getTargetPosition() > MAX_ARM_POS) {
            moveArm(Viper.MAX_VIPER_POS);
        } else if (armMotor.getTargetPosition() < MIN_ARM_POS) {
            moveArm(Viper.MIN_VIPER_POS);
        }

        if (wristServo.getPosition() > MAX_WRIST_POS) {
            moveWrist(MAX_WRIST_POS);
        } else if (wristServo.getPosition() < MIN_WRIST_POS) {
            moveWrist(MIN_WRIST_POS);
        }

        // vert pos is 155
        // move wrist to 0.95 at vert pos

        // pickup pos is 850
        // servo is 0.24

        // lift pos is 713
        // servo is 0

    }
}
